// Generated from training config; SI units.
#pragma once
namespace tree1 {
struct Config {
    double sample_spacing = 0.001;
    double lookahead_distance = 0.02;
    double goal_tolerance = 0.005;
    double rotate_threshold_deg = 75.0;
    int search_window = 3000;
    double dt = 0.05;
    double wheel_radius = 0.0325;
    double wheel_separation = 0.16;
    double max_wheel_rad_s = 12.0;
    double forward_speed = 0.1;
    double reverse_speed = 0.07;
    double max_angular_speed = 1.8;
    double max_acceleration = 0.5;
    double turn_gain = 3.0;
    double position_noise = 0.001;
    double yaw_noise = 0.01;
    int max_delay_steps = 2;
    int max_steps = 1200;
};
}
