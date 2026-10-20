#include <Wire.h>

#include "RobotWifiAP.h"

/** @brief WiFi access point for mouse-drawn routes, run commands and telemetry. */
RobotWifiAP_c server("myAP", "myPassword", 80);

#include "Robot.h"
#include "RobotIMU.h"
#include "Controller.h"
#include "AIPath.h"


/** @brief Hardware-facing interface to the Pololu robot. */
Robot_c robot;
/** @brief Direct interface to the 3Pi+ accelerometer, gyro, and magnetometer. */
RobotIMU_c imu;
/** @brief Two-tree controller; software START or the offline hardware button. */
Controller_c controller;


/** @brief Initialises serial, WiFi, IMU, and the two-tree path controller. */
void setup() {

  Serial.begin(115200);
  server.begin();

  robot.initialise();
//  robot.initialise("team name");

  robot.setMotorPWM(0, 0);
  if (!controller.setPath(ai_path::POINTS, ai_path::COUNT)) {
    Serial.printf("AI path rejected: %s\n", controller.lastError());
  }


  // Robot_c starts the shared I2C bus before RobotIMU_c uses it. IMU readings
  // come directly from the sensor ICs and do not pass through the middleware.
  if (!imu.initialise() ) {
    Serial.println("Warning: LSM6DS33 IMU was not detected.");
    robot.setLED( 90, 50, 0, 100 ); // orange warning
  }

}


/** @brief Maintains telemetry transport and runs the controller repeatedly. */
void loop() {
  // Process software route uploads before the timed control cycle. Once a
  // software client takes control, disconnect/heartbeat loss stops the run.
  server.update();

  
  // Run an iteration of the robot controller code.
  // Develop controller routines in Controller.cpp; edit parameters in Controller.h.
  controller.update(robot, server);

}
