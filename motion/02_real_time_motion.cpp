#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/robots/yaskawa_gp12.hpp>


using namespace jacobi;
using namespace jacobi::robots;

int main() {
    // 1. Set up the robot, transformations, and its kinematics limits
    auto robot = std::make_shared<YaskawaGP12>();
    robot->base() = Frame::from_translation(0.0, 0.0, 0.765);
    robot->flange_to_tcp() = Frame::from_translation(0.0, 0.0, 0.2);
    robot->max_velocity = {4.3, 3.4, 4.3, 7.0, 7.0, 9.0};
    robot->max_acceleration = {7.0, 7.0, 7.5, 15.0, 15.0, 18.0};
    robot->max_jerk = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    // 2. Set up the planner with the cycle time of the robot (alias the timer interval of trajectory steps)
    //    Here, we skip the environment and all collision checking, so all motion calculations are real-time
    //    capable and take usually less than 0.1ms in the on-prem version.
    auto planner = std::make_shared<Planner>(robot, 0.01); // [s]

    // [Cloud version] Authenticate with your account API key by setting
    // the `JACOBI_API_KEY` and `JACOBI_API_SECRET` environment variable.

    // 3. Define start and goal positions and plan the motion
    Config home_joint_position {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Config buffer_joint_position {-0.467, 0.2692, -0.3035, -0.1806, 0.0021, 0.0};

    const auto trajectory = planner->plan(home_joint_position, buffer_joint_position);

    // 4. Print returned trajectory
    if (!trajectory) {
        std::cout << "Could not find a trajectory!" << std::endl;
        return -1;
    }

    std::cout << "Trajectory duration: " << trajectory->duration << " [s]" << std::endl;
    std::cout << "Calculation duration: " << planner->last_calculation_duration << " [ms]" << std::endl;
}
