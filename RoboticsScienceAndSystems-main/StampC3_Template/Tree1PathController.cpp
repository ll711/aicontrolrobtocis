#include "Tree1PathController.h"

#include <cmath>
#include <limits>

namespace tree1_embedded {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDuplicateTolerance = 1e-12;

double wrap(double angle) {
    angle = std::fmod(angle + kPi, 2 * kPi);
    if (angle < 0) angle += 2 * kPi;
    return angle - kPi;
}

double distance(Point a, Point b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

bool finitePoint(Point point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}
}  // namespace

void Tree1PathController::reset() {
    progress_ = 0;
    previous_position_ = {};
    progress_credit_ = 0;
    have_previous_position_ = false;
    error_ = ready() ? "" : "no path loaded";
}

void Tree1PathController::clear() {
    count_ = 0;
    progress_ = 0;
    previous_position_ = {};
    progress_credit_ = 0;
    have_previous_position_ = false;
    error_ = "no path loaded";
}

bool Tree1PathController::failLoad(const char* message) {
    clear();
    error_ = message;
    return false;
}

bool Tree1PathController::append(Point point) {
    if (count_ >= kMaxResampledPoints) {
        return failLoad("path exceeds Tree 1 sample capacity");
    }
    if (!finitePoint(point)) return failLoad("nonfinite resampled path");
    double arc = count_ == 0 ? 0 : arc_[count_ - 1] + distance(path_[count_ - 1], point);
    if (!std::isfinite(arc)) return failLoad("nonfinite path length");
    path_[count_] = point;
    arc_[count_] = arc;
    ++count_;
    return true;
}

bool Tree1PathController::loadPath(const Point* points, std::size_t count,
                                  const tree1::Config& config,
                                  ProgressMode progressMode) {
    clear();
    if (!points || count < 2) return failLoad("path needs two distinct points");
    if (!std::isfinite(config.sample_spacing) || !(config.sample_spacing > 0) ||
        !std::isfinite(config.lookahead_distance) || !(config.lookahead_distance > 0) ||
        !std::isfinite(config.goal_tolerance) || !(config.goal_tolerance > 0) ||
        config.search_window <= 0) {
        return failLoad("invalid Tree 1 geometry configuration");
    }
    if (progressMode != ProgressMode::Continuous && progressMode != ProgressMode::SourceCompatible)
        return failLoad("invalid Tree 1 progress mode");
    for (std::size_t i = 0; i < count; ++i) {
        if (!finitePoint(points[i])) return failLoad("nonfinite path point");
    }
    config_ = config;
    progress_mode_ = progressMode;
    if (!append(points[0])) return false;

    // Stream the source runtime's duplicate removal and resampling into fixed
    // storage. Carry 'next' across vertices to preserve its geometry exactly.
    Point previous = points[0];
    double next = config_.sample_spacing;
    bool distinct = false;
    for (std::size_t i = 1; i < count; ++i) {
        const Point current = points[i];
        const double length = distance(previous, current);
        if (!std::isfinite(length)) return failLoad("nonfinite path segment");
        if (length <= kDuplicateTolerance) continue;
        distinct = true;
        double travelled = 0;
        while (travelled + next <= length) {
            travelled += next;
            const double fraction = travelled / length;
            if (!append({previous.x + fraction * (current.x - previous.x),
                         previous.y + fraction * (current.y - previous.y)})) {
                return false;
            }
            next = config_.sample_spacing;
        }
        next -= length - travelled;
        previous = current;
    }
    if (!distinct) return failLoad("path needs two distinct points");
    if (distance(path_[count_ - 1], previous) > kDuplicateTolerance && !append(previous)) {
        return false;
    }
    if (count_ < 2) return failLoad("path needs two distinct samples");
    reset();
    return true;
}

bool Tree1PathController::step(Pose pose, Output& output) {
    output = Output{};
    if (!ready()) {
        error_ = "no path loaded";
        return false;
    }
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw_ros)) {
        error_ = "nonfinite pose";
        return false;
    }

    const bool continuous = progress_mode_ == ProgressMode::Continuous;
    const double path_length = arc_[count_ - 1];
    const double sampling_margin = config_.sample_spacing < path_length / 2
        ? 2 * config_.sample_spacing : path_length;
    const double requested_credit = config_.lookahead_distance > sampling_margin
        ? config_.lookahead_distance : sampling_margin;
    const double credit_limit = requested_credit < path_length ? requested_credit : path_length;
    double allowance = credit_limit;
    if (continuous && have_previous_position_) {
        const double movement = distance(previous_position_, {pose.x, pose.y});
        // A factor of two accommodates cutting a corner and discrete sampling.
        // Unused credit is capped below, so stationary calls cannot progressively
        // walk the search window around a closed route to its endpoint.
        allowance = progress_credit_ + 2 * movement;
        if (!std::isfinite(allowance)) {
            error_ = "pose movement overflow";
            return false;
        }
    }

    std::size_t progress = progress_;
    const std::size_t available = count_ - progress_;
    const std::size_t window = static_cast<std::size_t>(config_.search_window);
    const std::size_t end = window >= available - 1 ? count_ : progress_ + window + 1;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = progress_; i < end; ++i) {
        if (continuous && have_previous_position_ &&
            arc_[i] - arc_[progress_] > allowance + kDuplicateTolerance) break;
        const double dx = path_[i].x - pose.x;
        const double dy = path_[i].y - pose.y;
        const double squared_distance = dx * dx + dy * dy;
        if (!std::isfinite(squared_distance)) {
            error_ = "pose/path distance overflow";
            return false;
        }
        // A global first search can mistake the end of a closed route for its
        // start, or select a later branch at a crossing. Locate the first local
        // minimum instead. Straight paths still support starting mid-route.
        if (continuous && !have_previous_position_ && squared_distance > best) break;
        if (squared_distance < best) {
            best = squared_distance;
            progress = i;
        }
    }

    std::size_t target = count_ - 1;
    double travelled = 0;
    for (std::size_t i = progress + 1; i < count_; ++i) {
        travelled += distance(path_[i - 1], path_[i]);
        if (travelled + kDuplicateTolerance >= config_.lookahead_distance) {
            target = i;
            break;
        }
    }
    const Point point = path_[target];
    const double vx = point.x - pose.x;
    const double vy = point.y - pose.y;
    const double d = std::hypot(vx, vy);
    const std::size_t tangent_index = progress < count_ - 2 ? progress : count_ - 2;
    double tx = path_[tangent_index + 1].x - path_[tangent_index].x;
    double ty = path_[tangent_index + 1].y - path_[tangent_index].y;
    const double length = std::hypot(tx, ty);
    if (length <= kDuplicateTolerance) {
        tx = 1;
        ty = 0;
    } else {
        tx /= length;
        ty /= length;
    }
    const double heading = -pose.yaw_ros;
    const double tangent = -std::atan2(ty, tx);
    const double tangent_error = wrap(tangent - heading);
    const bool geometric_reverse = std::abs(tangent_error) > kPi / 2;
    const double travel = d > kDuplicateTolerance ? -std::atan2(vy, vx) : tangent;
    const double heading_error = wrap(travel + (geometric_reverse ? kPi : 0) - heading);
    const double goal = distance({pose.x, pose.y}, path_[count_ - 1]);
    const double remaining = arc_[count_ - 1] - arc_[progress];
    Geometry geometry{};
    geometry.features = {{vx, vy, d, tx, ty, heading_error, tangent_error, goal,
                          remaining, std::abs(heading_error), std::abs(tangent_error)}};
    for (double feature : geometry.features) {
        if (!std::isfinite(feature)) {
            error_ = "nonfinite Tree 1 geometry";
            return false;
        }
    }
    geometry.target = point;
    geometry.progress_index = progress;
    geometry.target_index = target;
    geometry.heading_cw = heading;
    geometry.travel_heading_cw = travel;
    geometry.reverse = geometric_reverse;

    auto mode = tree1_model::predict(geometry.features);
    const bool terminal = goal <= config_.goal_tolerance && remaining <= config_.goal_tolerance;
    if (terminal) mode = tree1_model::Action::STOP;
    const bool reverse = mode == tree1_model::Action::REVERSE ||
        (mode == tree1_model::Action::ROTATE && geometric_reverse);
    double target_heading = wrap(travel + (reverse ? kPi : 0));
    double delta = wrap(target_heading - heading);
    if (mode == tree1_model::Action::STOP) {
        delta = 0;
        target_heading = heading;
    }
    output.mode = mode;
    output.signal = tree1_model::signal(mode);
    output.delta_theta = delta;
    output.target_heading_cw = target_heading;
    output.geometry = geometry;
    output.terminal = terminal;
    if (continuous) {
        if (!have_previous_position_) {
            progress_credit_ = credit_limit;
        } else {
            const double unused = allowance - (arc_[progress] - arc_[progress_]);
            progress_credit_ = unused < 0 ? 0 : (unused > credit_limit ? credit_limit : unused);
        }
        previous_position_ = {pose.x, pose.y};
        have_previous_position_ = true;
    }
    progress_ = progress;
    error_ = "";
    return true;
}

}  // namespace tree1_embedded
