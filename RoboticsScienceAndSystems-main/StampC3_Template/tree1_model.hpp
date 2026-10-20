// Generated from River HAT. Frozen inference; no online learning on device.
#pragma once
#include <array>
#include <cmath>
#include <cstddef>
namespace tree1_model {
enum class Action { FORWARD=0, REVERSE=1, ROTATE=2, STOP=3 };
inline constexpr int SCHEMA_VERSION = 2;
inline constexpr std::size_t FEATURE_COUNT = 11;
using Features = std::array<double, FEATURE_COUNT>;
inline constexpr const char* FEATURE_NAMES[] = {"vector_x","vector_y","distance","tangent_x","tangent_y","heading_error","tangent_heading_error","goal_distance","remaining_distance","abs_heading_error","abs_tangent_heading_error"};
inline constexpr const char* ACTION_NAMES[] = {"FORWARD","REVERSE","ROTATE","STOP"};
struct Node { int feature; double threshold; int left, right, alternate; std::array<double,4> scores; };
inline constexpr Node NODES[] = {
  {10, 2.243878038872928, 1, 18, -1, {{0,0,0,0}}},
  {2, 0.0036251926605271852, 2, 3, -1, {{0,0,0,0}}},
  {-1, 0, -1, -1, -1, {{0,0,0,1}}},
  {9, 1.3362108558053243, 4, 13, -1, {{0,0,0,0}}},
  {10, 1.3864558300755778, 5, 10, -1, {{0,0,0,0}}},
  {2, 0.0070053861073157723, 6, 9, -1, {{0,0,0,0}}},
  {2, 0.0050751159363244692, 7, 8, -1, {{0,0,0,0}}},
  {-1, 0, -1, -1, -1, {{0.013422302465610405,0,0,0.98657769753438962}}},
  {-1, 0, -1, -1, -1, {{0.9650222436006215,0,0.034977756399378584,0}}},
  {-1, 0, -1, -1, -1, {{55007.471416271321,0,281.63808786115987,0}}},
  {10, 1.5510913576570915, 11, 12, -1, {{0,0,0,0}}},
  {-1, 0, -1, -1, -1, {{2700.1773462985975,0,40.896250710147648,41.488902991256545}}},
  {-1, 0, -1, -1, -1, {{6.3958024439822188,402.18674978755257,10.942945669992977,15.168440923767928}}},
  {2, 0.006874510948061123, 14, 17, -1, {{0,0,0,0}}},
  {2, 0.0050465709837447092, 15, 16, -1, {{0,0,0,0}}},
  {-1, 0, -1, -1, -1, {{0,0,1.236301369863013,89.013698630136943}}},
  {-1, 0, -1, -1, -1, {{0,0,1,0}}},
  {-1, 0, -1, -1, -1, {{0,0,1,0}}},
  {2, 0.0066228966659690573, 19, 22, -1, {{0,0,0,0}}},
  {2, 0.0046503251390424202, 20, 21, -1, {{0,0,0,0}}},
  {-1, 0, -1, -1, -1, {{0,0,0,1}}},
  {-1, 0, -1, -1, -1, {{0,2.9992074754121636,1.2151961322790663,1.9902244914823266}}},
  {9, 1.2985575303363499, 23, 24, -1, {{0,0,0,0}}},
  {-1, 0, -1, -1, -1, {{0,0.99999999999999989,0,0}}},
  {-1, 0, -1, -1, -1, {{0,35.44752136752134,1244.6018613485269,0}}},
};
inline constexpr int PRIORITY[] = {3,2,1,0};
inline void vote(int index, const Features& x, std::array<double,4>& scores) {
    const Node& n = NODES[index];
    if (n.feature < 0) {
        for (int i=0; i<4; ++i) scores[i] += n.scores[i];
        return;
    }
    if (n.alternate >= 0) vote(n.alternate, x, scores);
    vote(x[n.feature] <= n.threshold ? n.left : n.right, x, scores);
}
inline Action predict(const Features& x) {
    for (double v : x) if (!std::isfinite(v)) return Action::STOP;
    std::array<double,4> scores{};
    vote(0, x, scores);
    double total=0; for (double s : scores) total += s;
    if (!(total > 0)) return Action::STOP;
    int best = PRIORITY[0];
    for (int i : PRIORITY) if (scores[i] > scores[best]) best=i;
    return static_cast<Action>(best);
}
inline const char* name(Action a) { return ACTION_NAMES[static_cast<int>(a)]; }
inline int signal(Action a) { return a==Action::FORWARD ? 1 : a==Action::REVERSE ? -1 : 0; }
}  // namespace tree1_model
