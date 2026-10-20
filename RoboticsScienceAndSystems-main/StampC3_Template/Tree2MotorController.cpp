#include "Tree2MotorController.h"
#include "Tree2Model.h"

#include <cmath>

namespace tree2_motor {
namespace {
double clamp(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
}
std::int16_t pwm(double value) {
    return static_cast<std::int16_t>(std::lround(clamp(value, -PWM_LIMIT, PWM_LIMIT)));
}
}  // namespace

bool predict(const tree1_embedded::Output& decision,
             std::int16_t previousLeft, std::int16_t previousRight,
             Command& command) {
    command = {};
    using Action = tree1_model::Action;
    if (decision.mode == Action::STOP) return true;
    if (!std::isfinite(decision.delta_theta)) return false;

    // Use the same shortest clockwise angle for both the learned moving modes
    // and the explicit rotation controller. Python maps the -pi tie to +pi.
    constexpr double pi = 3.14159265358979323846;
    double angle = std::fmod(decision.delta_theta, 2.0 * pi);
    if (angle <= -pi) angle += 2.0 * pi;
    if (angle > pi) angle -= 2.0 * pi;

    if (decision.mode == Action::ROTATE) {
        // Positive clockwise error requires left wheel faster than right.
        const double turn = clamp(45.0 * angle,
                                  -ROTATE_PWM_LIMIT, ROTATE_PWM_LIMIT);
        command = {pwm(turn), pwm(-turn)};
        return true;
    }
    if (decision.mode != Action::FORWARD && decision.mode != Action::REVERSE)
        return false;
    if (previousLeft < -PWM_LIMIT || previousLeft > PWM_LIMIT ||
        previousRight < -PWM_LIMIT || previousRight > PWM_LIMIT) return false;

    const double goal = decision.geometry.features[7];
    const double remaining = decision.geometry.features[8];
    if (!std::isfinite(goal) || !std::isfinite(remaining) || goal < 0 || remaining < 0)
        return false;

    const std::int8_t direction = decision.mode == Action::FORWARD ? 1 : -1;
    const float difference = Tree2Model::predict(static_cast<float>(angle),
                                                direction, previousLeft, previousRight);
    if (!std::isfinite(difference)) return false;

    const double distanceToGo = goal > remaining ? goal : remaining;
    const double base = (direction > 0 ? FORWARD_PWM : -REVERSE_PWM)
        * clamp(std::cos(angle), 0.0, 1.0)
        * clamp(distanceToGo / tree1::Config{}.lookahead_distance, 0.0, 1.0);
    // Reserve equal headroom around the requested base. Clipping each wheel
    // independently would change the average forward/reverse speed when a
    // linear leaf predicts beyond the training teacher's saturation boundary.
    const double differentialLimit = 2.0 * (PWM_LIMIT - std::abs(base));
    const double differential = clamp(difference, -differentialLimit, differentialLimit);
    command = {pwm(base - differential / 2.0), pwm(base + differential / 2.0)};
    return true;
}
}  // namespace tree2_motor
