#pragma once
// Minimal host-only Arduino declarations needed by odometry and TaskTimer.
#include <cmath>
#include <cstdint>
using std::isfinite;
inline constexpr double TWO_PI = 6.28318530717958647692;
inline unsigned long millis() { return 0; }
struct HostSerial {
    explicit operator bool() const { return false; }
    template <class... Args> void printf(const char*, Args...) {}
};
inline HostSerial Serial;
