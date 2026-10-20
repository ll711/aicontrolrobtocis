// Desktop validation; this file is outside the Arduino sketch source directory.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "Tree1PathController.h"
#include "Tree2MotorController.h"

// Separate namespaces let the original checkout supply an independent model
// and geometry implementation, without sharing the embedded copies' types.
namespace reference {
#include TREE1_REFERENCE_RUNTIME
}

namespace {
using Action = tree1_model::Action;
using Embedded = tree1_embedded::Tree1PathController;
using ProgressMode = tree1_embedded::ProgressMode;
static Embedded embedded;  // The firmware controller also belongs in static storage.
int checks = 0;

void require(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void close(double actual, double expected, const char* message) {
    require(std::isfinite(actual) && std::isfinite(expected), message);
    require(std::abs(actual - expected) <= 1e-11 * (1.0 + std::abs(expected)), message);
}

void compare(const tree1_embedded::Output& actual, const reference::tree1::Output& expected) {
    require(static_cast<int>(actual.mode) == static_cast<int>(expected.mode), "Tree 1 action parity");
    require(actual.signal == expected.signal, "Tree 1 signal parity");
    close(actual.delta_theta, expected.delta_theta, "Tree 1 heading error parity");
    close(actual.target_heading_cw, expected.target_heading_cw, "Tree 1 target heading parity");
    require(actual.geometry.progress_index == expected.geometry.progress_index, "Tree 1 progress parity");
    require(actual.geometry.target_index == expected.geometry.target_index, "Tree 1 lookahead index parity");
    require(actual.geometry.reverse == expected.geometry.reverse, "Tree 1 geometric reverse parity");
    close(actual.geometry.target.x, expected.geometry.target.x, "Tree 1 target x parity");
    close(actual.geometry.target.y, expected.geometry.target.y, "Tree 1 target y parity");
    close(actual.geometry.heading_cw, expected.geometry.heading_cw, "Tree 1 current heading parity");
    close(actual.geometry.travel_heading_cw, expected.geometry.travel_heading_cw, "Tree 1 travel heading parity");
    for (std::size_t i = 0; i < actual.geometry.features.size(); ++i)
        close(actual.geometry.features[i], expected.geometry.features[i], "Tree 1 feature parity");
}

void parityChecks() {
    const std::vector<std::vector<tree1_embedded::Point>> paths = {
        {{0, 0}, {0.2, 0}},
        {{0, 0}, {0.1, 0}, {0.1, 0.1}},
        {{0, 0}, {0, 0}, {0.08, 0.02}, {0.08, 0.02}, {0.02, -0.04}},
        {{0.03, -0.07}, {0.09, 0.05}, {-0.02, 0.06}, {0.03, -0.07}},
        {{0, 0}, {0.0000000000001, 0}, {0.02, 0}, {0.019, 0.001}, {0.2, 0.02}},
        {{0, 0}, {0.0005, 0.0001}}
    };
    std::mt19937 random(20261018);
    std::uniform_real_distribution<double> coordinate(-0.2, 0.3);
    std::uniform_real_distribution<double> yaw(-15.0, 15.0);
    for (int configuration = 0; configuration < 3; ++configuration) {
        tree1::Config config;
        reference::tree1::Config reference_config;
        if (configuration == 1) {
            config.sample_spacing = reference_config.sample_spacing = 0.0037;
            config.lookahead_distance = reference_config.lookahead_distance = 0.018;
            config.search_window = reference_config.search_window = 3;
        } else if (configuration == 2) {
            config.sample_spacing = reference_config.sample_spacing = 0.0007;
            config.lookahead_distance = reference_config.lookahead_distance = 0.009;
            config.goal_tolerance = reference_config.goal_tolerance = 0.001;
            config.search_window = reference_config.search_window = 1;
        }
        for (const auto& path : paths) {
            std::vector<reference::tree1::Point> reference_path;
            for (const auto point : path) reference_path.push_back({point.x, point.y});
            require(embedded.loadPath(path.data(), path.size(), config, ProgressMode::SourceCompatible),
                    "source-compatible path accepted");
            require(embedded.sampleCount() == reference::tree1::resample(reference_path, config.sample_spacing).size(),
                    "resampled count parity");
            reference::tree1::Controller source(reference_path, reference_config);
            for (int i = 0; i < 300; ++i) {
                const tree1_embedded::Pose pose{coordinate(random), coordinate(random), yaw(random)};
                tree1_embedded::Output actual;
                require(embedded.step(pose, actual), "finite pose accepted");
                compare(actual, source.step({pose.x, pose.y, pose.yaw_ros}));
            }
            embedded.reset();
            reference::tree1::Controller restarted(reference_path, reference_config);
            tree1_embedded::Output actual;
            require(embedded.step({path.front().x, path.front().y, 0}, actual), "reset preserves route");
            compare(actual, restarted.step({path.front().x, path.front().y, 0}));
        }
    }
}

void continuousProgressChecks() {
    using Point = tree1_embedded::Point;
    tree1_embedded::Output out;
    const Point square[]{{0, 0}, {0.2, 0}, {0.2, 0.2}, {0, 0.2}, {0, 0}};

    // A 1 mm initial Y error lies on the final edge of this closed route. The
    // original global window jumps there and stops without traversing the loop.
    require(embedded.loadPath(square, 5, {}, ProgressMode::SourceCompatible), "legacy closed path loads");
    require(embedded.step({0, 0.001, 0}, out), "legacy noisy closed start observed");
    require(out.terminal, "reproduce original closed-route premature completion");
    require(embedded.loadPath(square, 5), "continuous closed path loads by default");
    require(embedded.step({0, 0.001, 0}, out), "continuous noisy closed start observed");
    require(out.geometry.progress_index == 0 && !out.terminal && out.mode != Action::STOP,
            "closed-route start stays on its first branch");
    for (int i = 0; i < 300; ++i) {
        require(embedded.step({0, 0.001, 0}, out), "stationary closed pose remains valid");
        require(out.geometry.progress_index == 0 && !out.terminal,
                "stationary calls do not accumulate forward search credit");
    }

    auto trace = [&](const Point* path, std::size_t count) {
        require(embedded.loadPath(path, count), "continuous trace loads");
        require(embedded.step({path[0].x, path[0].y, 0}, out), "trace starts");
        std::size_t previous = out.geometry.progress_index;
        for (std::size_t edge = 1; edge < count; ++edge) {
            const Point a = path[edge - 1], b = path[edge];
            const double dx = b.x - a.x, dy = b.y - a.y;
            const int steps = static_cast<int>(std::ceil(std::hypot(dx, dy) / 0.005));
            for (int sample = 1; sample <= steps; ++sample) {
                const double fraction = static_cast<double>(sample) / steps;
                require(embedded.step({a.x + fraction * dx, a.y + fraction * dy, std::atan2(dy, dx)}, out),
                        "continuous route observation accepted");
                require(out.geometry.progress_index >= previous, "continuous progress never goes backwards");
                require(out.geometry.progress_index - previous <= 32,
                        "5 mm motion cannot skip a distant route branch");
                if (edge + 1 < count || sample + 2 < steps)
                    require(!out.terminal && out.mode != Action::STOP, "route continues before its final approach");
                previous = out.geometry.progress_index;
            }
        }
        require(out.terminal && out.mode == Action::STOP, "complete continuous route stops at its true endpoint");
    };
    trace(square, 5);
    embedded.reset();
    require(embedded.step({0, 0.001, 0}, out) && out.geometry.progress_index == 0 && !out.terminal,
            "reset clears motion history as well as route progress");

    const Point crossing[]{{0, 0}, {0.2, 0.2}, {0, 0.2}, {0.2, 0}, {0.4, 0}};
    require(embedded.loadPath(crossing, 5, {}, ProgressMode::SourceCompatible), "legacy crossing path loads");
    require(embedded.step({0, 0, 0}, out) && embedded.step({0.1001, 0.0999, 0.7853981633974483}, out),
            "legacy crossing observations accepted");
    require(out.geometry.progress_index > 500, "reproduce original crossing branch jump");
    require(embedded.loadPath(crossing, 5), "continuous crossing path loads");
    require(embedded.step({0, 0, 0}, out), "continuous crossing starts");
    for (int i = 1; i <= 19; ++i)
        require(embedded.step({i * 0.005, i * 0.005, 0.7853981633974483}, out),
                "approach crossing along first branch");
    require(embedded.step({0.1001, 0.0999, 0.7853981633974483}, out), "noisy crossing observation accepted");
    require(out.geometry.progress_index >= 135 && out.geometry.progress_index < 160,
            "crossing keeps the first branch despite a closer later branch");
    const std::size_t at_crossing = out.geometry.progress_index;
    for (int i = 0; i < 300; ++i) {
        require(embedded.step({0.1001, 0.0999, 0.7853981633974483}, out), "stationary crossing observation accepted");
        require(out.geometry.progress_index == at_crossing, "stationary crossing cannot walk to another branch");
    }
    trace(crossing, 5);

    // Initial localization on an unambiguous straight path keeps the existing
    // ability to start partway along the route or recognize its endpoint.
    const Point straight[]{{0, 0}, {0.2, 0}};
    require(embedded.loadPath(straight, 2), "continuous straight path loads");
    require(embedded.step({0.1, 0, 0}, out) && out.geometry.progress_index >= 99 && out.mode == Action::FORWARD,
            "initial straight-path localization supports a midway start");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    require(!embedded.step({nan, 0, 0}, out), "invalid observation rejected during continuous tracking");
    require(embedded.step({0.105, 0, 0}, out) && out.geometry.progress_index >= 104,
            "invalid observation does not corrupt motion history");
    require(!embedded.loadPath(straight, 2, {}, static_cast<ProgressMode>(99)),
            "invalid progress mode rejected");
}

void inputAndCapacityChecks() {
    const tree1_embedded::Point route[]{{0, 0}, {0.1, 0}};
    tree1_embedded::Output output;
    embedded.clear();
    require(!embedded.step({}, output) && output.mode == Action::STOP, "no path fails stopped");
    require(!embedded.loadPath(nullptr, 2), "null path rejected");
    require(!embedded.loadPath(route, 1), "single point rejected");
    const tree1_embedded::Point duplicates[]{{0, 0}, {0, 0}, {0, 0}};
    require(!embedded.loadPath(duplicates, 3), "duplicate-only path rejected");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    const tree1_embedded::Point invalid[]{{0, 0}, {nan, 0}};
    require(!embedded.loadPath(invalid, 2), "nonfinite path rejected");
    for (int field = 0; field < 4; ++field) {
        tree1::Config config;
        if (field == 0) config.sample_spacing = 0;
        if (field == 1) config.lookahead_distance = nan;
        if (field == 2) config.goal_tolerance = -1;
        if (field == 3) config.search_window = 0;
        require(!embedded.loadPath(route, 2, config), "invalid geometry configuration rejected");
    }
    require(embedded.loadPath(route, 2), "valid route before invalid pose");
    for (const auto pose : {tree1_embedded::Pose{nan, 0, 0}, {0, infinity, 0}, {0, 0, nan}}) {
        output.mode = Action::FORWARD;
        output.signal = 1;
        output.delta_theta = 1;
        require(!embedded.step(pose, output), "nonfinite pose rejected");
        require(output.mode == Action::STOP && output.signal == 0 && output.delta_theta == 0,
                "pose failure clears stale output");
        require(embedded.ready(), "invalid pose preserves loaded route");
    }
    require(embedded.step({0.1, 0, 0}, output), "endpoint accepted");
    require(output.mode == Action::STOP && output.terminal, "exact endpoint stops");
    require(output.signal == 0 && output.delta_theta == 0, "endpoint command is stopped");

    tree1::Config large;
    large.sample_spacing = 1;
    const double capacity = static_cast<double>(Embedded::kMaxResampledPoints);
    const tree1_embedded::Point exact[]{{0, 0}, {capacity - 1, 0}};
    require(embedded.loadPath(exact, 2, large), "exact capacity accepted");
    require(embedded.sampleCount() == Embedded::kMaxResampledPoints, "exact capacity sample count");
    const tree1_embedded::Point excessive[]{{0, 0}, {capacity, 0}};
    require(!embedded.loadPath(excessive, 2, large), "capacity overflow rejected");
    require(!embedded.ready() && embedded.sampleCount() == 0, "failed load unloads previous route");
    require(!embedded.step({}, output) && output.mode == Action::STOP, "capacity failure remains stopped");
    const double maximum = std::numeric_limits<double>::max();
    tree1::Config sparse;
    sparse.sample_spacing = maximum;
    require(embedded.loadPath(route, 2, sparse), "maximum finite sample spacing accepts finite short route");
    require(embedded.step({0, 0, 0}, output), "maximum spacing keeps continuous progress arithmetic finite");
    require(embedded.step({0.1, 0, 0}, output) && output.terminal,
            "sparsely sampled short route can still reach its endpoint");
    const tree1_embedded::Point overflow[]{{-maximum, 0}, {maximum, 0}};
    require(!embedded.loadPath(overflow, 2), "distance overflow rejected");
}

tree1_embedded::Output decision(Action action, double delta) {
    tree1_embedded::Output out;
    out.mode = action;
    out.signal = tree1_model::signal(action);
    out.delta_theta = delta;
    out.geometry.features[7] = 0.1;
    out.geometry.features[8] = 0.1;
    return out;
}

void bounded(const tree2_motor::Command& command) {
    require(command.left >= -100 && command.left <= 100 && command.right >= -100 && command.right <= 100,
            "motor command within conservative PWM bounds");
}

void motorChecks() {
    tree2_motor::Command command;
    require(tree2_motor::predict(decision(Action::STOP, 2), 100, -100, command), "STOP accepted");
    require(command.left == 0 && command.right == 0, "STOP bypasses model and previous command");
    for (const double delta : {-3.0, -0.8, 0.8, 3.0}) {
        require(tree2_motor::predict(decision(Action::ROTATE, delta), 75, 75, command), "ROTATE accepted");
        require(command.left == -command.right, "ROTATE has no translation");
        require(delta > 0 ? command.left > 0 : command.left < 0, "clockwise-positive turn convention");
        require(std::abs(command.left) <= 35, "rotation limit");
    }
    for (const auto action : {Action::FORWARD, Action::REVERSE}) {
        require(tree2_motor::predict(decision(action, 0), 0, 0, command), "translation accepted");
        require(command.left == command.right, "straight start has no learned steering bias");
        require(action == Action::FORWARD ? command.left + command.right > 0 : command.left + command.right < 0,
                "forward and reverse translation signs");
        for (const double small_angle : {0.02, 0.05, 0.1}) {
            tree2_motor::Command clockwise, counterclockwise;
            require(tree2_motor::predict(decision(action, small_angle), 0, 0, clockwise),
                    "small clockwise turn accepted");
            require(tree2_motor::predict(decision(action, -small_angle), 0, 0, counterclockwise),
                    "small counterclockwise turn accepted");
            // At 0.02 rad the requested differential is about -0.63 PWM;
            // rounding each wheel to an integer can produce equal outputs.
            require(clockwise.left >= clockwise.right, "small clockwise error never turns the wrong way");
            if (small_angle >= 0.05)
                require(clockwise.left > clockwise.right, "resolvable clockwise error turns clockwise");
            require(counterclockwise.left == clockwise.right && counterclockwise.right == clockwise.left,
                    "opposite errors swap the two motor commands");
        }
        for (int previous = -100; previous <= 100; previous += 25) {
            for (int angle = -180; angle <= 180; angle += 3) {
                const double delta = angle * 3.14159265358979323846 / 180.0;
                require(tree2_motor::predict(decision(action, delta), static_cast<int16_t>(previous),
                                            static_cast<int16_t>(-previous), command), "finite model prediction");
                bounded(command);
            }
        }
    }
    auto invalid = decision(Action::FORWARD, std::numeric_limits<double>::quiet_NaN());
    command = {30, 40};
    require(!tree2_motor::predict(invalid, 0, 0, command), "nonfinite heading rejected");
    require(command.left == 0 && command.right == 0, "invalid prediction clears stale motor command");
    invalid = decision(Action::FORWARD, 0);
    invalid.geometry.features[7] = std::numeric_limits<double>::infinity();
    require(!tree2_motor::predict(invalid, 0, 0, command), "nonfinite distance rejected");
    require(command.left == 0 && command.right == 0, "invalid distance remains stopped");
    invalid = decision(Action::FORWARD, 0);
    invalid.geometry.features[8] = -0.1;
    require(!tree2_motor::predict(invalid, 0, 0, command), "negative remaining distance rejected");
    require(!tree2_motor::predict(decision(Action::FORWARD, 0), 101, 0, command),
            "out-of-range previous PWM rejected");
    require(!tree2_motor::predict(decision(static_cast<Action>(99), 0), 0, 0, command),
            "invalid action rejected");
    require(command.left == 0 && command.right == 0, "invalid action remains stopped");

    const tree1_embedded::Point route[]{{0, 0}, {0.2, 0}};
    struct Case { tree1_embedded::Pose pose; Action mode; };
    const Case cases[] = {
        {{0, 0, 0}, Action::FORWARD},
        {{0, 0, 3.14159265358979323846}, Action::REVERSE},
        {{0, 0, 1.57079632679489661923}, Action::ROTATE},
        {{0.2, 0, 0}, Action::STOP}
    };
    for (const auto& item : cases) {
        require(embedded.loadPath(route, 2), "combined controller loads route");
        tree1_embedded::Output out;
        require(embedded.step(item.pose, out), "combined controller observes pose");
        require(out.mode == item.mode, "combined controller exercises expected Tree 1 mode");
        require(tree2_motor::predict(out, 0, 0, command), "Tree 1 output accepted by Tree 2 motor adapter");
        bounded(command);
        if (item.mode == Action::STOP)
            require(command.left == 0 && command.right == 0, "combined endpoint stops both motors");
    }
}
}  // namespace

int main() {
    parityChecks();
    continuousProgressChecks();
    inputAndCapacityChecks();
    motorChecks();
    std::cout << "PASS: " << checks << " checks (Tree 1 source parity and continuous routes, invalid input/capacity, Tree 2 motor semantics)\n";
    return 0;
}
