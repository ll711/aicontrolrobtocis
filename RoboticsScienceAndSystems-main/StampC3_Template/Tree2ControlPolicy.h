#pragma once
#include "Tree2MeasuredModel.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace tree2_measured {
inline constexpr float DEADBAND_RAD = 0.00628318531f;  // 0.36 degrees
inline constexpr float GAIN = 90.0f;                  // PWM / rad
inline constexpr float TEACHER_TARGET_LIMIT = 140.0f;
inline constexpr float REVERSE_HISTORY_WEIGHT = 0.65f;

inline constexpr std::size_t LEAF_COUNT = sizeof(LEAVES) / sizeof(LEAVES[0]);

// A one-feature CART partitions the line into ordered intervals.
inline std::size_t leafIndex(float angle) {
    std::size_t lo = 0, hi = LEAF_COUNT - 1;
    while (lo < hi) {
        const auto mid = lo + (hi - lo) / 2;
        if (angle > LEAVES[mid].upper) lo = mid + 1;
        else hi = mid;
    }
    return lo;
}

// Interpolate the active CART leaf toward its neighbor. The origin is the straight
// equilibrium and the endpoint leaf means are held constant outside support.
inline float interpolate(float angle) {
    if (angle <= LEAVES[0].center) return LEAVES[0].difference;
    if (angle >= LEAVES[LEAF_COUNT-1].center) return LEAVES[LEAF_COUNT-1].difference;
    const auto active = leafIndex(angle);
    const auto lo = angle < LEAVES[active].center ? active - 1 : active;
    auto a = LEAVES[lo], b = LEAVES[lo+1];
    if (a.center < 0 && b.center > 0) {
        if (angle < 0) b = {};
        else a = {};
    }
    const float t = (angle - a.center) / (b.center - a.center);
    return a.difference + t * (b.difference - a.difference);
}

// The motor adapter validates inputs, wraps the angle and handles STOP/ROTATE first.
inline float predict(float angle, std::int8_t direction, std::int16_t prevL, std::int16_t prevR) {
    const float magnitude = std::abs(angle);
    if (magnitude <= DEADBAND_RAD) return 0.0f;
    const float sign = angle > 0 ? -1.0f : 1.0f;
    const float envelope = GAIN * (magnitude - DEADBAND_RAD);
    if (direction > 0)
        return sign * std::clamp(sign * interpolate(angle), 0.0f, envelope);

    // No measured reverse episodes. Retain the documented synthetic teacher's
    // reverse target and lag, admitting only history aligned with the current turn.
    const float target = std::min(envelope, TEACHER_TARGET_LIMIT);
    const float history = std::clamp(sign * (prevR - prevL), 0.0f, target);
    return sign * (target + REVERSE_HISTORY_WEIGHT * (history - target));
}

// Under proportional wheel speeds, curvature is proportional to difference/base.
// If the measured steering falls short of the nominal angular request, reduce
// translation by the same ratio. Keep full straight speed in the deadband.
inline float forwardSpeedScale(float angle, float difference) {
    const float nominal = std::min(GAIN * (std::abs(angle) - DEADBAND_RAD), TEACHER_TARGET_LIMIT);
    const float actual = std::abs(difference);
    return actual < nominal ? actual / nominal : 1.0f;
}
}  // namespace tree2_measured
