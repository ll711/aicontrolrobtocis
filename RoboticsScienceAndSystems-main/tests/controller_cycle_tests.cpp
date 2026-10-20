#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <Arduino.h>
#include "Tree2Model.h"

// Supply hardware boundary fakes, then compile the real controller unchanged.
// The production Robot implementation is validated by the firmware build.
#define ROBOT_H
#define ROBOT_WIFI_AP_H
class Robot_c {
public:
    unsigned long now = 0;
    bool pressed = false;
    bool encoder_ok = true;
    bool motor_ok = true;
    int32_t left_count = 0, right_count = 0;
    int16_t left_pwm = 0, right_pwm = 0;
    int acquisitions = 0;
    struct { uint16_t reading[5]{}; } surface;
    std::vector<std::pair<int16_t, int16_t>> writes;
    unsigned long getMillis() const { return now; }
    bool getSurfaceSensors() { return true; }
    bool getEncoders() { ++acquisitions; return encoder_ok; }
    bool getPose() { return true; }
    bool isButtonPressed() const { return pressed; }
    int32_t getLeftEncoderCount() const { return left_count; }
    int32_t getRightEncoderCount() const { return right_count; }
    int16_t getLeftMotorPWM() const { return left_pwm; }
    int16_t getRightMotorPWM() const { return right_pwm; }
    bool setMotorPWM(float left, float right) {
        writes.push_back({static_cast<int16_t>(left), static_cast<int16_t>(right)});
        if (!motor_ok) return false;
        left_pwm = static_cast<int16_t>(left);
        right_pwm = static_cast<int16_t>(right);
        return true;
    }
};
class RobotWifiAP_c {
public:
    bool connected = false;
    uint32_t generation = 0;
    std::string incoming;
    std::size_t read_offset = 0;
    std::string fail_send_prefix;
    std::vector<std::string> records;
    bool clientConnected() const { return connected; }
    uint32_t connectionGeneration() const { return generation; }
    std::size_t available() const { return connected ? incoming.size() - read_offset : 0; }
    int read() {
        return available() ? static_cast<unsigned char>(incoming[read_offset++]) : -1;
    }
    void receive(const std::string& bytes) {
        if (!connected) ++generation;
        connected = true;
        incoming += bytes;
    }
    void disconnect() {
        connected = false;
        incoming.clear();
        read_offset = 0;
    }
    void disconnectClient() { disconnect(); }
    bool send(const char* text) {
        if (!connected) return false;
        records.emplace_back(text);
        return true;
    }
    bool sendLine(const char* text) { return send((std::string(text) + "\n").c_str()); }
    int printf(const char* format, ...) {
        char buffer[1024];
        va_list arguments;
        va_start(arguments, format);
        const int count = std::vsnprintf(buffer, sizeof(buffer), format, arguments);
        va_end(arguments);
        records.emplace_back(buffer);
        if (!fail_send_prefix.empty() && records.back().compare(0, fail_send_prefix.size(), fail_send_prefix) == 0) {
            fail_send_prefix.clear();
            disconnect();  // Real nonblocking transport closes on failed/partial send.
            return 0;
        }
        return count;
    }
};
#include "Controller.cpp"

namespace {
int checks = 0;
void require(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
void close(double actual, double expected, const char* message) {
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-6, message);
}
bool isTelemetry(const std::string& record) {
    return !record.empty() && record[0] >= '0' && record[0] <= '9' &&
           std::count(record.begin(), record.end(), ',') == 13;
}
std::size_t telemetryCount(const RobotWifiAP_c& server) {
    return static_cast<std::size_t>(std::count_if(server.records.begin(), server.records.end(), isTelemetry));
}
const tree1_embedded::Point route[]{{0, 0}, {0.2, 0}};
struct Fixture {
    Controller_c controller;
    Robot_c robot;
    RobotWifiAP_c server;
    Fixture() { require(controller.setPath(route, 2), "controller accepts valid route"); }
    void cycle(unsigned long now) {
        robot.now = now;
        controller.update(robot, server);
    }
    void start() { controller.setSignal(1); }
    void wire(const std::string& text, unsigned long now) {
        server.receive(text);
        cycle(now);
    }
    void requireStopped(const char* message) const {
        require(robot.left_pwm == 0 && robot.right_pwm == 0, message);
    }
    void requireForward(const char* message) const {
        require(robot.left_pwm + robot.right_pwm > 0, message);
    }
};

void cadenceAndButton() {
    Fixture f;
    require(f.controller.getSignal() == 0, "loaded controller waits for start");
    f.cycle(9);
    require(f.robot.acquisitions == 0 && f.robot.writes.empty(), "10 ms acquisition interval enforced");
    f.cycle(10);
    f.requireStopped("waiting cycle sends zero PWM");
    require(f.robot.acquisitions == 1, "ready cycle acquires encoders");
    f.robot.pressed = true;
    f.cycle(20);
    require(f.controller.getSignal() == 1, "button starts loaded controller");
    f.robot.pressed = false;
    f.cycle(30);
    f.cycle(40);
    f.requireStopped("model waits for 50 ms cadence");
    f.cycle(50);
    f.requireForward("Tree 1 and Tree 2 command motion at first ready cycle");
    const std::size_t writes = f.robot.writes.size();
    f.cycle(60);
    require(f.robot.writes.size() == writes, "no redundant model update between 50 ms ticks");
    f.cycle(100);
    require(f.robot.writes.size() == writes + 1, "next 50 ms tick writes model output");
    require(telemetryCount(f.server) == 2, "telemetry retains 50 ms interval and 14 CSV columns");
    f.controller.setSignal(0);
    f.cycle(110);
    f.requireStopped("reset stops at next acquisition cycle");
}

void baselineAndUnits() {
    Fixture f;
    f.robot.left_count = f.robot.right_count = 15000;
    f.controller.odometry.reset(100, 0, 0);
    f.start();
    f.cycle(50);
    f.requireStopped("first absolute encoder snapshot establishes baseline without moving");
    require(f.controller.getSignal() == 1, "first baseline does not latch a fault");
    f.cycle(60);
    f.requireForward("second encoder snapshot permits inference");
    close(f.controller.odometry.pose.x, 100, "absolute counts do not displace reset pose");
    close(f.controller.lastDecision().geometry.features[7], 0.1, "millimetre pose converted to metre geometry");
    close(f.controller.lastDecision().geometry.target.x, 0.12, "lookahead remains in metre units");

    f.controller.setSignal(0);
    f.controller.odometry.reset(50, 0, 0);
    f.start();
    f.cycle(110);
    f.requireStopped("new trial establishes a fresh encoder baseline");
    f.cycle(120);
    f.requireForward("explicit odometry reset and controller restart resumes correctly");
    close(f.controller.lastDecision().geometry.features[7], 0.15, "restart uses newly reset pose");
}

void headingAndEndpoint() {
    for (const float yaw : {-1.4f, 1.4f}) {
        Fixture f;
        f.controller.odometry.reset(0, 0, yaw);
        f.start();
        f.cycle(50);
        f.cycle(60);
        require(f.controller.lastDecision().mode == tree1_model::Action::ROTATE,
                "large path heading error requests ROTATE");
        require(yaw > 0 ? f.robot.left_pwm > 0 && f.robot.right_pwm < 0
                        : f.robot.left_pwm < 0 && f.robot.right_pwm > 0,
                "CCW odometry yaw correctly reaches clockwise motor convention");
    }
    Fixture f;
    f.controller.odometry.reset(200, 0, 0);
    f.start();
    f.cycle(50);
    f.cycle(60);
    f.requireStopped("route endpoint stops motors");
    require(f.controller.lastDecision().terminal, "endpoint retains geometric terminal marker");
    f.controller.odometry.pose.x = 0;
    f.controller.setSignal(1);
    f.cycle(110);
    f.requireStopped("endpoint stop stays latched until reset");
    require(f.controller.getSignal() == 1, "telemetry start signal retained after stop");
}

void failureLatching() {
    Fixture encoder;
    encoder.start();
    encoder.cycle(50);
    encoder.cycle(60);
    encoder.requireForward("normal run before encoder failure");
    encoder.robot.encoder_ok = false;
    encoder.cycle(70);
    encoder.requireStopped("encoder acquisition failure stops immediately");
    require(std::string(encoder.controller.lastError()).find("encoder") != std::string::npos,
            "encoder failure is reported");
    encoder.robot.encoder_ok = true;
    encoder.controller.setSignal(1);
    encoder.cycle(110);
    encoder.requireStopped("successful sensor read cannot silently clear latched fault");

    Fixture motor;
    motor.start();
    motor.cycle(50);
    motor.robot.motor_ok = false;
    const std::size_t before = motor.robot.writes.size();
    motor.cycle(60);
    require(motor.robot.writes.size() == before + 2, "failed motion command attempts immediate stop");
    require(motor.robot.writes.back() == std::make_pair<int16_t, int16_t>(0, 0),
            "failed motor write is followed by a zero command");
    require(std::string(motor.controller.lastError()) == "motor write failed", "motor failure is reported");
    motor.robot.motor_ok = true;
    motor.cycle(70);
    motor.requireStopped("latched motor fault retries stop on next acquisition");
    motor.controller.setSignal(1);
    motor.cycle(110);
    motor.requireStopped("motor fault requires explicit reset");

    Fixture odometry;
    odometry.start();
    odometry.cycle(50);
    odometry.controller.odometry.leftWheelRadiusMm = 0;
    odometry.cycle(60);
    odometry.requireStopped("invalid odometry calibration stops motion");
    require(std::string(odometry.controller.lastError()).find("odometry") != std::string::npos,
            "odometry update failure is reported");
}

void routeReplacement() {
    Fixture f;
    f.start();
    f.cycle(50);
    f.cycle(60);
    require(!f.controller.setPath(nullptr, 0), "invalid replacement route rejected");
    require(!f.controller.pathReady() && f.controller.getSignal() == 0,
            "invalid route unloads old route and disarms controller");
    f.controller.setSignal(1);
    f.cycle(70);
    f.requireStopped("invalid route cannot reuse previous motor commands");
    require(f.controller.getSignal() == 0, "missing route cannot start");
}

const char* uploaded_route = "ROUTE_BEGIN 3\nROUTE_POINT 0 0\nROUTE_POINT 0.3 0\nROUTE_POINT 0.3 0.2\nROUTE_END\n";
bool hasRecord(const RobotWifiAP_c& server, const std::string& prefix) {
    return std::any_of(server.records.begin(), server.records.end(), [&](const std::string& value) {
        return value.compare(0, prefix.size(), prefix) == 0;
    });
}

void remoteUploadStartStopAndReset() {
    Fixture f;
    f.controller.odometry.reset(100, 0, 0);
    f.robot.left_count = f.robot.right_count = 16000;
    f.wire(uploaded_route, 5);
    require(f.controller.remoteRouteReady(), "complete network upload arms the committed route");
    require(!f.controller.isRunning(), "upload does not start physical motion");
    f.requireStopped("upload keeps both motors stopped");
    require(hasRecord(f.server, "ACK ROUTE_END"), "committed route is acknowledged");
    require(hasRecord(f.server, "STATUS mode=WAITING route_ready=1"), "status announces committed waiting route");
    f.wire("START\n", 6);
    require(f.controller.isRunning(), "START is accepted for a committed remote route");
    f.cycle(10);
    f.requireStopped("remote start establishes encoder baseline without moving");
    f.cycle(50);
    f.requireForward("remote route reaches the two-tree motor pipeline");
    close(f.controller.lastDecision().geometry.features[7], std::hypot(0.2, 0.2),
          "uploaded metre coordinates combine correctly with millimetre odometry");
    f.wire("STOP\n", 51);
    f.requireStopped("network STOP is immediate before 10 ms acquisition gate");
    require(!f.controller.isRunning() && f.controller.getSignal() == 0, "network STOP disarms controller");
    require(f.controller.remoteRouteReady(), "explicit STOP retains committed route for restart");
    f.wire("RESET_POSE\n", 52);
    close(f.controller.odometry.pose.x, 0, "RESET_POSE zeros local x");
    close(f.controller.odometry.pose.y, 0, "RESET_POSE zeros local y");
    close(f.controller.odometry.pose.theta, 0, "RESET_POSE zeros local yaw");
    f.wire("START\n", 53);
    f.cycle(60);
    f.requireStopped("remote pose reset establishes a new encoder baseline");
    f.cycle(100);
    f.requireForward("remote restart runs from reset pose");
    close(f.controller.odometry.pose.x, 0, "absolute counts do not shift remotely reset pose");
    close(f.controller.lastDecision().geometry.features[7], std::hypot(0.3, 0.2),
          "reset route keeps expected world geometry");
    require(telemetryCount(f.server) == 2, "command/status records preserve CSV telemetry cadence");
}

void remoteOwnershipAndReconnect() {
    Fixture f;
    f.start();
    f.cycle(50);
    f.cycle(60);
    f.requireForward("local run before remote ownership");
    f.wire("PING\n", 61);
    f.requireStopped("new connection immediately takes ownership and stops local run");
    require(!f.controller.remoteRouteReady(), "new remote session cannot start old local route");
    f.robot.pressed = true;
    f.cycle(70);
    require(!f.controller.isRunning(), "hardware button cannot start remotely owned controller");
    f.wire("START\n", 71);
    require(!f.controller.isRunning() && hasRecord(f.server, "ERR START"), "remote START requires full upload");
    f.robot.pressed = false;
    f.wire(std::string(uploaded_route) + "RESET_POSE\nSTART\n", 72);
    f.cycle(80);
    f.cycle(110);
    f.requireForward("uploaded remote route starts after local takeover");
    f.server.disconnect();
    f.cycle(111);
    f.requireStopped("disconnect stops motion before next acquisition tick");
    require(!f.controller.remoteRouteReady(), "disconnect revokes committed remote route readiness");
    f.robot.pressed = true;
    f.cycle(120);
    require(!f.controller.isRunning(), "button remains disabled after remote disconnect");
    f.wire("START\n", 121);
    require(!f.controller.isRunning(), "reconnect requires a fresh complete upload");
    f.wire("ROUTE_BEGIN 2\nROUTE_POINT 0 0\nROUTE_POINT 0.2", 122);
    f.server.disconnect();
    f.cycle(123);
    f.wire(" 0\nROUTE_END\n", 124);
    require(!f.controller.remoteRouteReady() && !f.controller.isRunning(),
            "disconnected partial line and staged route cannot leak into a new session");

    f.wire(std::string(uploaded_route) + "START\n", 125);
    f.cycle(135);
    f.cycle(185);
    f.requireForward("fresh upload recovers from rejected reconnect fragments");
    ++f.server.generation;  // A socket replacement can happen between controller polls.
    f.cycle(186);
    f.requireStopped("new connection generation stops without needing an observed disconnect");
    require(!f.controller.remoteRouteReady(), "socket generation replacement revokes old route");
}

void remoteHeartbeatAndInputBounds() {
    Fixture f;
    f.wire(std::string(uploaded_route) + "START\n", 5);
    f.cycle(10);
    f.cycle(50);
    f.requireForward("normal remote run before heartbeat exercise");
    f.wire("PING\n", 2500);
    require(f.controller.isRunning(), "accepted heartbeat preserves active remote run");
    require(hasRecord(f.server, "ACK PING"), "heartbeat has distinct acknowledgment");
    f.cycle(5499);
    require(f.controller.isRunning(), "heartbeat allows a full timeout interval");
    f.cycle(5500);
    f.requireStopped("missing heartbeat stops at the 3000 ms deadline");
    require(!f.controller.remoteRouteReady(), "heartbeat timeout requires route upload again");
    require(std::string(f.controller.lastError()).find("heartbeat") != std::string::npos,
            "heartbeat timeout is visible to software");
    f.wire("PING\nSTART\n", 5501);
    require(!f.controller.isRunning(), "late heartbeat cannot silently restart expired route");

    Fixture flood;
    std::string pings;
    for (int i = 0; i < 200; ++i) pings += "PING\n";
    flood.wire(pings, 10);
    require(flood.server.read_offset <= 512 && flood.server.available() > 0,
            "network processing bounds work per controller call");
    require(flood.robot.acquisitions == 1, "queued network traffic does not starve encoder acquisition");
    flood.cycle(20);
    require(flood.server.available() == 0 && flood.robot.acquisitions == 2,
            "bounded network input drains across subsequent control cycles");
}

void remoteRejectedUploadAndMotorFailure() {
    Fixture f;
    f.wire(std::string(uploaded_route) + "START\n", 5);
    f.cycle(10);
    f.cycle(50);
    f.requireForward("normal run before malformed remote upload");
    f.wire("ROUTE_BEGIN 2\nROUTE_POINT 0 0\nROUTE_POINT nan 0\nROUTE_END\nSTART\n", 51);
    f.requireStopped("malformed replacement upload stops immediately");
    require(!f.controller.remoteRouteReady() && !f.controller.isRunning(),
            "rejected upload cannot rearm previous route");
    f.wire(uploaded_route, 52);
    require(f.controller.remoteRouteReady() && !f.controller.isRunning(),
            "fresh complete upload recovers from rejected protocol");
    f.robot.motor_ok = false;
    f.wire("RESET_POSE\n", 53);
    require(!f.controller.remoteRouteReady() && !f.controller.isRunning(),
            "failed commanded stop rejects pose reset and revokes readiness");
    require(hasRecord(f.server, "ERR RESET_POSE"), "stop write failure is not acknowledged as a successful pose reset");
    f.robot.motor_ok = true;
    f.cycle(60);
    f.requireStopped("failed network stop is retried during subsequent control cycle");

    Fixture encoder;
    encoder.wire(std::string(uploaded_route) + "START\n", 5);
    encoder.cycle(10);
    encoder.cycle(50);
    encoder.robot.encoder_ok = false;
    encoder.cycle(60);
    require(!encoder.controller.remoteRouteReady() && !encoder.controller.isRunning(),
            "encoder fault revokes remote readiness so software requires re-upload");
    encoder.requireStopped("remote encoder fault stops motors");
}

void motorAngleAndSaturationRegressions() {
    tree1_embedded::Output output;
    output.mode = tree1_model::Action::ROTATE;
    tree2_motor::Command actual, wrapped;
    constexpr double pi = 3.14159265358979323846;
    output.delta_theta = 0.2;
    require(tree2_motor::predict(output, 0, 0, wrapped), "small rotation accepted");
    output.delta_theta = 0.2 + 2 * pi;
    require(tree2_motor::predict(output, 0, 0, actual), "wrapped rotation accepted");
    require(actual.left == wrapped.left && actual.right == wrapped.right,
            "ROTATE normalizes turns to shortest clockwise angle");
    output.mode = tree1_model::Action::FORWARD;
    output.geometry.features[7] = output.geometry.features[8] = 1;
    for (const auto mode : {tree1_model::Action::FORWARD, tree1_model::Action::REVERSE}) {
        output.mode = mode;
        for (const double angle : {-0.5, -0.2, 0.0, 0.2, 0.5}) {
            output.delta_theta = angle;
            const double base = (mode == tree1_model::Action::FORWARD ? 45 : -35) * std::cos(angle);
            for (const int previous : {-100, 100}) {
                require(tree2_motor::predict(output, static_cast<int16_t>(previous),
                                            static_cast<int16_t>(-previous), actual), "extreme previous difference accepted");
                require(std::abs((actual.left + actual.right) / 2.0 - base) <= 0.5,
                        "saturated differential preserves requested average translation speed");
                require(std::abs(actual.left) <= 100 && std::abs(actual.right) <= 100,
                        "saturated differential keeps both motors inside PWM bounds");
            }
        }
    }
}

void remoteSendFailures() {
    Fixture starting;
    starting.wire(uploaded_route, 5);
    starting.cycle(10);
    starting.server.fail_send_prefix = "ACK START";
    const std::size_t before = starting.robot.writes.size();
    starting.wire("START\n", 50);
    starting.requireStopped("failed START acknowledgment stops during the same update");
    require(!starting.controller.isRunning() && !starting.controller.remoteRouteReady(),
            "failed START acknowledgment revokes remote run and route");
    require(std::all_of(starting.robot.writes.begin() + before, starting.robot.writes.end(),
                       [](const auto& pwm) { return pwm.first == 0 && pwm.second == 0; }),
            "no motion command follows an unacknowledged START");

    Fixture status;
    status.wire(std::string(uploaded_route) + "START\n", 5);
    status.cycle(10);
    status.cycle(50);
    status.requireForward("normal run before status send failure");
    status.server.fail_send_prefix = "STATUS";
    status.cycle(255);
    status.requireStopped("failed status transmission stops in the same update");
    require(!status.controller.isRunning() && !status.controller.remoteRouteReady(),
            "status send failure cancels remotely armed route");

    Fixture telemetry;
    telemetry.wire(std::string(uploaded_route) + "START\n", 5);
    telemetry.cycle(10);
    telemetry.cycle(50);
    telemetry.requireForward("normal run before telemetry send failure");
    telemetry.server.fail_send_prefix = "100,";
    telemetry.cycle(100);
    telemetry.requireStopped("failed telemetry transmission stops in the same update");
    require(!telemetry.controller.isRunning() && !telemetry.controller.remoteRouteReady(),
            "telemetry send failure revokes remote readiness");
}

void tree2ZeroErrorAndMirrorRegressions() {
    for (const int direction : {-1, 1}) {
        for (const int previous : {-100, -50, 0, 50, 100}) {
            const float delta = Tree2Model::predict(0, static_cast<int8_t>(direction),
                                                   static_cast<int16_t>(previous), static_cast<int16_t>(previous));
            require(std::isfinite(delta) && std::abs(delta) <= 1e-6f,
                    "equal previous wheel PWM and zero heading produce zero learned steering bias");
        }
        for (const float angle : {0.001f, 0.01f, 0.1f, 0.5f}) {
            const float clockwise = Tree2Model::predict(angle, static_cast<int8_t>(direction), 0, 0);
            const float counterclockwise = Tree2Model::predict(-angle, static_cast<int8_t>(direction), 0, 0);
            require(clockwise < 0 && counterclockwise > 0, "small heading errors steer in their requested direction");
            require(std::abs(clockwise + counterclockwise) <= 1e-5f,
                    "opposite small heading errors have symmetric learned steering");
            const float history = Tree2Model::predict(angle, static_cast<int8_t>(direction), 30, -20);
            const float mirror = Tree2Model::predict(-angle, static_cast<int8_t>(direction), -20, 30);
            require(std::abs(history + mirror) <= 1e-5f,
                    "mirroring heading and previous wheel difference mirrors learned output");
        }
        tree1_embedded::Output decision;
        decision.mode = direction > 0 ? tree1_model::Action::FORWARD : tree1_model::Action::REVERSE;
        decision.geometry.features[7] = decision.geometry.features[8] = 1;
        tree2_motor::Command command;
        require(tree2_motor::predict(decision, 0, 0, command), "zero-error translation accepted");
        require(command.left == command.right && command.left == (direction > 0 ? 45 : -35),
                "straight path starts with equal commanded wheel PWM");
    }
}
}  // namespace

int main() {
    cadenceAndButton();
    baselineAndUnits();
    headingAndEndpoint();
    failureLatching();
    routeReplacement();
    remoteUploadStartStopAndReset();
    remoteOwnershipAndReconnect();
    remoteHeartbeatAndInputBounds();
    remoteRejectedUploadAndMotorFailure();
    motorAngleAndSaturationRegressions();
    remoteSendFailures();
    tree2ZeroErrorAndMirrorRegressions();
    std::cout << "PASS: " << checks << " controller cycle checks using actual Controller.cpp and odometry\n";
    return 0;
}
