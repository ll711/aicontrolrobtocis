#pragma once

#include "Tree1PathController.h"
#include <cstdint>

namespace tree2_motor {

struct Command {
    std::int16_t left = 0;
    std::int16_t right = 0;
};

// PWM units, matching the synthetic Tree 2 teacher. Tune with measured data.
constexpr int PWM_LIMIT = 100;
constexpr double FORWARD_PWM = 45.0;
constexpr double REVERSE_PWM = 35.0;
constexpr double ROTATE_PWM_LIMIT = 35.0;

// Tree 2 predicts right-minus-left PWM, not two independent motor commands.
// STOP bypasses inference. ROTATE uses a separate signed rotation controller:
// the existing Tree 2 trainer excludes signal == 0 and cannot learn rotation.
// All moving modes use the shortest clockwise angle in (-pi, pi]. PWM
// difference is limited around the requested base to preserve average speed.
// Returns false and a zero command for invalid inputs or predictions.
bool predict(const tree1_embedded::Output& decision,
             std::int16_t previousLeft, std::int16_t previousRight,
             Command& command);

}  // namespace tree2_motor
