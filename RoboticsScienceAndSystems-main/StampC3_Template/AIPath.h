#pragma once
#include "Tree1PathController.h"

// Example route in metres, in the local odometry frame. At startup the robot
// is (0, 0), facing +X. This is only the offline button-start fallback.
// Normally draw and upload a new route in DigitalTwin without recompiling.
// 200 mm forward, then 150 mm towards +Y. The button starts the controller.
namespace ai_path {
inline constexpr tree1_embedded::Point POINTS[] = {
    {0.0, 0.0},
    {0.20, 0.0},
    {0.20, 0.15}
};
inline constexpr std::size_t COUNT = sizeof(POINTS) / sizeof(POINTS[0]);
}
