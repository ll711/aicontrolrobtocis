#include "Tree1PathController.h"
#include "Tree2MotorController.h"
#include <iostream>
#include <iomanip>
#include <vector>
static tree1_embedded::Tree1PathController controller;
int main() {
  char kind;
  std::cout << std::setprecision(17);
  while(std::cin >> kind) {
    if(kind=='R') {
      unsigned count; std::cin >> count;
      std::vector<tree1_embedded::Point> path(count);
      for(auto& p:path) std::cin >> p.x >> p.y;
      if(!controller.loadPath(path.data(),path.size())) return 1;
    } else {
      tree1_embedded::Pose pose; int left,right;
      std::cin >> pose.x >> pose.y >> pose.yaw_ros >> left >> right;
      tree1_embedded::Output d; tree2_motor::Command c;
      if(!controller.step(pose,d) || !tree2_motor::predict(d,left,right,c)) return 2;
      std::cout << int(d.mode) << ' ' << d.geometry.progress_index << ' ' << d.geometry.target_index
        << ' ' << d.delta_theta << ' ' << c.left << ' ' << c.right << ' ' << d.terminal;
      for(double f:d.geometry.features) std::cout << ' ' << f;
      std::cout << '\n';
    }
  }
}
