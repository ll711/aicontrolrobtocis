// Host-only streaming probe of the exported headers and the actual motor adapter.
#include "Tree2MotorController.h"
#include "Tree2ControlPolicy.h"
#include "Tree2Model.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    tree1_embedded::Output decision;
    decision.mode = tree1_model::Action::FORWARD;
    decision.geometry.features[7] = decision.geometry.features[8] = 0.1;
    tree2_motor::Command cmd;
    if (argc == 2 && std::string(argv[1]) == "--version") {
        std::cout << (TREE2_USE_LEGACY ? "legacy" : "measured") << '\n';
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--benchmark") {
        constexpr int calls = 1000000;
        int checksum = 0;
        const auto start = std::chrono::steady_clock::now();
        for (int i = 0; i < calls; ++i) {
            decision.delta_theta = (i % 3601 - 1800) * 0.00174532925199433;
            if (!tree2_motor::predict(decision, -10, 10, cmd)) return 1;
            checksum += cmd.right - cmd.left;
        }
        const std::chrono::duration<double, std::nano> elapsed = std::chrono::steady_clock::now() - start;
        std::cout << elapsed.count() / calls << " ns/call, " << calls
                  << " host forward motor adapter calls, checksum " << checksum << '\n';
        return 0;
    }
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout << std::setprecision(10);
    int signal, left, right;
    // Direct header outputs use a pre-wrapped angle; the adapter accepts full precision.
    while (std::cin >> decision.delta_theta >> signal >> left >> right) {
        if ((signal != 1 && signal != -1) || left < -100 || left > 100 || right < -100 || right > 100)
            return 2;
        decision.mode = signal > 0 ? tree1_model::Action::FORWARD : tree1_model::Action::REVERSE;
        if (!tree2_motor::predict(decision, left, right, cmd)) return 3;
        const auto a = static_cast<float>(decision.delta_theta);
        std::cout << Tree2Model::predict(a, signal, left, right) << ' '
                  << tree2_measured::predict(a, signal, left, right) << ' '
                  << cmd.left << ' ' << cmd.right << ' '
                  << tree2_measured::LEAVES[tree2_measured::leafIndex(a)].difference << '\n';
    }
    return std::cin.eof() ? 0 : 4;
}
