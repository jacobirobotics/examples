#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/robots/abb_yumi_irb14000.hpp>


using namespace jacobi;
using namespace jacobi::robots;

int main() {
    // 1. Set up the robot, transformations, and its kinematics limits
    auto yumi = std::make_shared<ABBYuMiIRB14000>();
    yumi->set_speed(0.2);

    // 2. Calculate forward and inverse kinematics of left and right arms
    const auto tcp_right = yumi->right.calculate_tcp({0, -0.3, 0, 0.2, 0, 0.1, 0});
    std::cout << "Right TCP Position: " << tcp_right.translation() << std::endl;

    const auto tcp_left = yumi->left.calculate_tcp({0, -0.4, 0, 0, 0, 0, 0});
    std::cout << "Left TCP Position: " << tcp_left.translation() << std::endl;

    const auto joint_position_right = yumi->right.inverse_kinematics(tcp_left);
    if (!joint_position_right) {
        std::cout << "Could not find a inverse solution!" << std::endl;
        return -1;
    }

    std::cout << "Right Joint Position to Reach Left TCP: ";
    for (auto cj: *joint_position_right) {
        std::cout << cj << ", ";
    }
    std::cout << std::endl;

    // 3. Check for collision
    auto environment = std::make_shared<Environment>(yumi);

    const bool in_collision = environment->check_collision({0, -0.2, 0, 0, 0, 0, 0, /**/ 0, 0.3, 0, 0, 0, 0, 0});
    std::cout << "In collision: " << in_collision << std::endl;

    // 4.1 Set up the joint planner that considers left <-> right collisions
    auto planner = std::make_shared<Planner>(environment);

    // [Cloud version] Authenticate with your account API key by setting
    // the `JACOBI_API_KEY` and `JACOBI_API_SECRET` environment variable.

    const auto trajectory = planner->plan(
        {0, -0.3, 0, 0, 0, 0, 0, /**/  0, 0.3, 0, 0, 0, 0, 0},   // start in [rad]
        {0, 0.3, 0, 0.2, 0, 0, 0, /**/ 0, -0.5, 0, 0.4, 0, 0, 0} // goal in [rad]
    );

    // 4.2 Set up the planner for a single robot arm (without left <-> right collision checking)
    // Planner planner_left {&yumi.left};

    // const auto trajectory = planner_left.plan(
    //     {0, -0.3, 0, 0, 0, 0, 0},  // start in [rad]
    //     {0, 0.3, 0, 0.2, 0, 0, 0}  // goal in [rad]
    // );

    // 5. Print returned trajectory
    if (!trajectory) {
        std::cout << "Could not find a trajectory!" << std::endl;
        return -1;
    }

    std::cout << "Trajectory duration: " << trajectory->duration << " [s]" << std::endl;
    std::cout << "Calculation duration: " << planner->last_calculation_duration << " [ms]" << std::endl;
}
