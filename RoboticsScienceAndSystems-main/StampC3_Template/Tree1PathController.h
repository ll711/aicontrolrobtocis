#pragma once

#include "tree1_config.hpp"
#include "tree1_model.hpp"
#include <cstddef>

// The trained spacing is 1 mm. This capacity permits about 4.095 m of sampled
// path and uses 96 KiB for points and cumulative distances. Store this controller
// in static/global storage, not on an Arduino task's stack.
#ifndef TREE1_MAX_RESAMPLED_POINTS
#define TREE1_MAX_RESAMPLED_POINTS 4096
#endif

namespace tree1_embedded {

struct Point {
    double x = 0;
    double y = 0;
};

struct Pose {
    double x = 0;
    double y = 0;
    double yaw_ros = 0;  // Metres and radians; counterclockwise positive yaw.
};

struct Geometry {
    tree1_model::Features features{};
    Point target{};
    std::size_t progress_index = 0;
    std::size_t target_index = 0;
    double heading_cw = 0;
    double travel_heading_cw = 0;
    bool reverse = false;
};

struct Output {
    tree1_model::Action mode = tree1_model::Action::STOP;
    int signal = 0;
    double delta_theta = 0;        // Clockwise positive radians, for Tree 2.
    double target_heading_cw = 0;  // Clockwise positive radians.
    Geometry geometry{};
    // Exact geometric terminal condition. A learned STOP can also occur before
    // this condition. Both STOP cases must stop the motors; ROTATE must not.
    bool terminal = false;
};

enum class ProgressMode {
    // Follow route order at crossings. The first observation uses the first
    // local nearest location; later progress is bounded by observed movement.
    Continuous,
    // Original forward-window nearest search, for source/export verification.
    SourceCompatible
};

// Bounded-memory adapter for first_tree/cpp/tree1_runtime.hpp. Frozen inference,
// resampling and features are unchanged; Continuous protects progress on closed
// and crossing routes. No heap allocation, exceptions or Arduino APIs are used.
class Tree1PathController {
public:
    static constexpr std::size_t kMaxResampledPoints = TREE1_MAX_RESAMPLED_POINTS;
    static_assert(kMaxResampledPoints >= 2, "Tree 1 needs at least two path samples");

    // Invalid input or capacity exhaustion unloads the previous path. Adjacent
    // duplicate vertices are removed before resampling, as in the source model.
    bool loadPath(const Point* points, std::size_t count,
                  const tree1::Config& config = {},
                  ProgressMode progressMode = ProgressMode::Continuous);
    void reset();  // Restart progress without unloading the route.
    void clear();  // Unload the route.

    // Failure returns false and writes a zero-initialized STOP output.
    bool step(Pose pose, Output& output);
    bool ready() const { return count_ >= 2; }
    std::size_t sampleCount() const { return count_; }
    const char* lastError() const { return error_; }

private:
    Point path_[kMaxResampledPoints]{};
    double arc_[kMaxResampledPoints]{};
    std::size_t count_ = 0;
    std::size_t progress_ = 0;
    tree1::Config config_{};
    ProgressMode progress_mode_ = ProgressMode::Continuous;
    Point previous_position_{};
    double progress_credit_ = 0;
    bool have_previous_position_ = false;
    const char* error_ = "no path loaded";

    bool append(Point point);
    bool failLoad(const char* message);
};

}  // namespace tree1_embedded
