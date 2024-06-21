#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/robots/yaskawa_gp12.hpp>


using namespace jacobi;
using namespace jacobi::robots;

int main() {
    // 1. Set up the robot, and define the base frame as well as the flange to TCP transformation
    YaskawaGP12 robot;
    robot.base() = Frame::from_translation(0.0, 0.0, 0.765);
    robot.flange_to_tcp() = Frame::from_translation(0.0, 0.0, 0.2);

    // 2. Forward Kinematics
    Config home_config {-1.57, -0.7042, -0.4648, 0.0, -1.809, 0.0};
    Frame home_pose = robot.calculate_tcp(home_config);

    std::cout << "Home translation: " << home_pose.translation() << std::endl;

    // 3. Inverse Kinematics
    Frame grasp_pose = Frame::from_euler(0.4, 0.1, 0.4, 0.0, 0.0, -1.2);
    Config nearest_joint_position {-1.57, -0.7042, -0.4648, 0.0, -1.809, 0.0};

    const auto grasp_joint_position = robot.inverse_kinematics(grasp_pose, nearest_joint_position);
    if (!grasp_joint_position) {
        std::cout << "Could not find a inverse solution!" << std::endl;
        return -1;
    }

    std::cout << "Joint configuration: ";
    for (auto cj: *grasp_joint_position) {
        std::cout << cj << ", ";
    }
    std::cout << std::endl;
}
