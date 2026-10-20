#ifndef CONTROLLER_H
#define CONTROLLER_H

// Include models before Arduino headers to avoid min/max macro collisions.
#include "Tree1PathController.h"
#include "Tree2MotorController.h"
#include "RouteCommandParser.h"
#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"

/** Path -> frozen HAT action/angle -> Tree 2 PWM difference -> motors. */
class Controller_c {
public:
    OdometryModel_c odometry;
    static const unsigned long UPDATE_INTERVAL_MS = 10;
    static const unsigned long MODEL_INTERVAL_MS = 50;
    static const unsigned long TELEMETRY_INTERVAL_MS = 50;

    Controller_c();
    // Copies/resamples a route in metres. Resets to WAITING; odometry is kept.
    // Invalid routes unload the old route. Call while stationary, then start.
    bool setPath(const tree1_embedded::Point* points, std::size_t count);
    bool pathReady() const { return path_controller.ready(); }
    const tree1_embedded::Output& lastDecision() const { return decision; }
    const char* lastError() const { return error; }
    bool isRunning() const { return mode == FOLLOWING_PATH; }
    // Hardware/model faults and lost remote sessions revoke this flag. A STOP
    // command alone preserves a successfully uploaded route for an explicit restart.
    bool remoteRouteReady() const { return remote_route_ready && pathReady(); }

    // Restarts path progress and returns to WAITING. The next update sends zero
    // PWM; call robot.setMotorPWM(0, 0) as well when an immediate stop is needed.
    // Odometry remains unchanged; reset it explicitly before a new trial.
    void reset();
    uint8_t getSignal() const;
    // Zero resets; nonzero starts a loaded path only from WAITING. STOPPED is
    // latched (including I2C faults) until an explicit reset and new start.
    void setSignal(uint8_t requested_signal);
    void update(Robot_c& robot, RobotWifiAP_c& server);

private:
    enum Mode { WAITING, FOLLOWING_PATH, STOPPED };
    uint8_t signal;
    Mode mode;
    bool encoder_baseline_seen = false;
    const char* error = "";
    tree1_embedded::Tree1PathController path_controller;
    tree1_embedded::Output decision{};
    TaskTimer_c update_timer;
    TaskTimer_c model_timer;
    TaskTimer_c telemetry_timer;

    // Network editing owns the run after the first client connects. A hardware
    // button cannot restart a stopped/disconnected remote session.
    route_commands::Parser command_parser;
    bool remote_owned = false;
    bool remote_connected = false;
    bool remote_route_ready = false;
    uint32_t remote_generation = 0;
    unsigned long last_remote_command_ms = 0;
    unsigned long last_status_ms = 0;
    class RemoteSink;
    bool stopRemote(Robot_c& robot);
    void loseRemoteConnection(Robot_c& robot, RobotWifiAP_c& server, const char* reason);
    void serviceRemote(Robot_c& robot, RobotWifiAP_c& server, unsigned long now);
    bool publishStatus(RobotWifiAP_c& server);

    void runDecisionTrees(Robot_c& robot);
    void stopWithError(Robot_c& robot, const char* message);
    // Original 14 CSV columns: x/y in mm, theta CCW radians. Final signal is
    // start state, not the model direction; inspect lastDecision() for direction.
    void publishTelemetry(Robot_c& robot, RobotWifiAP_c& server, unsigned long timestamp_ms);
};
#endif
