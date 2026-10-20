#include "Controller.h"

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::MODEL_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;

class Controller_c::RemoteSink final : public route_commands::Sink {
public:
    RemoteSink(Controller_c& owner, Robot_c& hardware) : controller(owner), robot(hardware) {}
    bool stop() override { return controller.stopRemote(robot); }
    bool commitRoute(const tree1_embedded::Point* points, std::size_t count) override {
        // The parser validates all points and the 4 m length bound first. The
        // default 4096-sample Tree 1 buffer therefore fits the entire route.
        const bool loaded = controller.setPath(points, count);
        controller.remote_route_ready = loaded;
        return loaded;
    }
    bool start() override {
        if (!controller.remoteRouteReady() || controller.mode != WAITING) return false;
        controller.setSignal(1);
        return controller.isRunning();
    }
    bool resetPose() override {
        controller.odometry.reset();
        controller.encoder_baseline_seen = false;
        return true;
    }
private:
    Controller_c& controller;
    Robot_c& robot;
};

bool Controller_c::stopRemote(Robot_c& robot) {
    remote_owned = true;
    if (!robot.setMotorPWM(0, 0)) {
        stopWithError(robot, "motor stop write failed");
        return false;
    }
    reset();
    return true;
}

void Controller_c::loseRemoteConnection(Robot_c& robot, RobotWifiAP_c& server, const char* reason) {
    command_parser.disconnect();
    remote_route_ready = false;
    remote_connected = false;
    server.disconnectClient();
    stopRemote(robot);
    if (error[0] == '\0') error = reason;
}

void Controller_c::serviceRemote(Robot_c& robot, RobotWifiAP_c& server, unsigned long now) {
    const bool connected = server.clientConnected();
    const uint32_t generation = server.connectionGeneration();
    bool changed = false;
    if (connected && (!remote_connected || generation != remote_generation)) {
        command_parser.disconnect();
        remote_route_ready = false;
        stopRemote(robot);
        remote_connected = true;
        remote_generation = generation;
        last_remote_command_ms = now;
        changed = true;
    } else if (!connected && remote_connected) {
        loseRemoteConnection(robot, server, "remote disconnected; upload route again");
    }
    if (!connected) return;

    RemoteSink sink(*this, robot);
    for (std::size_t bytes = 0; bytes < 512 && server.available() > 0; ++bytes) {
        const int incoming = server.read();
        if (incoming < 0) break;
        const auto result = command_parser.feed(static_cast<char>(incoming), sink);
        if (result == route_commands::Result::None) continue;
        if (result == route_commands::Result::Accepted) {
            last_remote_command_ms = now;
        } else {
            remote_route_ready = false;
            stopRemote(robot);
            if (error[0] == '\0') error = "remote command rejected; upload route again";
        }
        if (!server.printf("%s\n", command_parser.lastResponse())) {
            // In particular, a START that cannot be acknowledged must not reach
            // the motor-control portion of this same update as a running state.
            loseRemoteConnection(robot, server, "remote response write failed; upload route again");
            return;
        }
        changed = true;
    }
    if ((isRunning() || remote_route_ready || command_parser.uploading()) &&
        now - last_remote_command_ms >= 3000) {
        command_parser.disconnect();
        remote_route_ready = false;
        stopRemote(robot);
        if (error[0] == '\0') error = "remote heartbeat timeout; upload route again";
        changed = true;
    }
    if (changed || now - last_status_ms >= 250) {
        if (!publishStatus(server)) {
            loseRemoteConnection(robot, server, "remote status write failed; upload route again");
            return;
        }
        last_status_ms = now;
    }
}

bool Controller_c::publishStatus(RobotWifiAP_c& server) {
    const char* action = decision.mode == tree1_model::Action::FORWARD ? "FORWARD" :
        decision.mode == tree1_model::Action::REVERSE ? "REVERSE" :
        decision.mode == tree1_model::Action::ROTATE ? "ROTATE" : "STOP";
    return server.printf("STATUS mode=%s route_ready=%d action=%s progress=%u points=%u error=%s\n",
        isRunning() ? "RUNNING" : mode == STOPPED ? "STOPPED" : "WAITING",
        remoteRouteReady() && !command_parser.uploading() ? 1 : 0, action,
        static_cast<unsigned>(decision.geometry.progress_index),
        static_cast<unsigned>(path_controller.sampleCount()), error[0] ? error : "none");
}

Controller_c::Controller_c()
    : update_timer(UPDATE_INTERVAL_MS), model_timer(MODEL_INTERVAL_MS),
      telemetry_timer(TELEMETRY_INTERVAL_MS) {
    reset();
}

bool Controller_c::setPath(const tree1_embedded::Point* points, std::size_t count) {
    remote_route_ready = false;
    reset();
    const bool loaded = path_controller.loadPath(points, count);
    if (!loaded) error = path_controller.lastError();
    return loaded;
}

void Controller_c::reset() {
    signal = 0;
    mode = WAITING;
    encoder_baseline_seen = false;
    decision = {};
    error = "";
    path_controller.reset();
}

uint8_t Controller_c::getSignal() const { return signal; }

void Controller_c::setSignal(uint8_t requested_signal) {
    if (requested_signal == 0) {
        reset();
    } else if (signal == 0) {
        if (!path_controller.ready()) {
            error = path_controller.lastError();
            return;
        }
        signal = 1;
        mode = FOLLOWING_PATH;
        error = "";
    }
}

void Controller_c::stopWithError(Robot_c& robot, const char* message) {
    mode = STOPPED;
    // A failed hardware/model cycle cannot resume a previously acknowledged
    // remote route. Publish route_ready=0 and require a new complete upload,
    // matching reconnect/heartbeat failure and the GUI's Start gate.
    remote_route_ready = false;
    decision = {};
    error = message;
    robot.setMotorPWM(0, 0); // Failed bus writes are retried on following cycles.
}

void Controller_c::update(Robot_c& robot, RobotWifiAP_c& server) {
    const unsigned long now = robot.getMillis();
    serviceRemote(robot, server, now);
    if (!update_timer.isReady(now)) return;
    update_timer.resetTimer(now);

    robot.getSurfaceSensors(); // Retained for the existing telemetry format.
    const bool encoders_ok = robot.getEncoders();
    const bool first_encoder_sample = encoders_ok && !encoder_baseline_seen;
    bool pose_updated = false;
    if (encoders_ok) {
        pose_updated = odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
        encoder_baseline_seen = true;
    }
    robot.getPose(); // Diagnostic cache only; AI uses the local encoder pose.

    if (!remote_owned && signal == 0 && robot.isButtonPressed()) setSignal(1);

    if (mode == FOLLOWING_PATH) {
        if (!encoders_ok || (!pose_updated && !first_encoder_sample)) {
            stopWithError(robot, "encoder acquisition or odometry failed");
        } else if (first_encoder_sample) {
            // The first absolute encoder snapshot only establishes a baseline.
            if (!robot.setMotorPWM(0, 0)) stopWithError(robot, "motor write failed");
        } else if (model_timer.isReady(now)) {
            model_timer.resetTimer(now);
            runDecisionTrees(robot);
        }
    } else {
        if (!robot.setMotorPWM(0, 0)) error = "motor stop write failed";
    }

    if (telemetry_timer.isReady(now)) {
        telemetry_timer.resetTimer(now);
        publishTelemetry(robot, server, now);
    }
}

void Controller_c::runDecisionTrees(Robot_c& robot) {
    // Local odometry is in millimetres; Tree 1 was trained in metres.
    // Both pose interfaces use CCW-positive yaw. Do not negate theta here.
    const tree1_embedded::Pose pose = {
        odometry.pose.x * 0.001, odometry.pose.y * 0.001, odometry.pose.theta
    };
    if (!path_controller.step(pose, decision)) {
        stopWithError(robot, path_controller.lastError());
        return;
    }

    tree2_motor::Command command;
    if (!tree2_motor::predict(decision, robot.getLeftMotorPWM(),
                             robot.getRightMotorPWM(), command)) {
        stopWithError(robot, "invalid Tree 2 input or output");
        return;
    }
    if (decision.mode == tree1_model::Action::STOP) mode = STOPPED;
    if (!robot.setMotorPWM(command.left, command.right)) {
        stopWithError(robot, "motor write failed");
    }
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  const bool sent = server.printf(
    "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
    timestamp_ms,
    odometry.pose.x,
    odometry.pose.y,
    odometry.pose.theta,
    robot.getLeftMotorPWM(),
    robot.getRightMotorPWM(),
    (long)robot.getLeftEncoderCount(),
    (long)robot.getRightEncoderCount(),
    robot.surface.reading[0],
    robot.surface.reading[1],
    robot.surface.reading[2],
    robot.surface.reading[3],
    robot.surface.reading[4],
    signal
  );
  if (remote_connected && !sent) {
    loseRemoteConnection(robot, server, "remote telemetry write failed; upload route again");
  }
  if ( Serial ) { // If serial is connected
    Serial.printf(
      "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
      timestamp_ms,
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta,
      robot.getLeftMotorPWM(),
      robot.getRightMotorPWM(),
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),
      robot.surface.reading[0],
      robot.surface.reading[1],
      robot.surface.reading[2],
      robot.surface.reading[3],
      robot.surface.reading[4],
      signal
    );
  }
}
