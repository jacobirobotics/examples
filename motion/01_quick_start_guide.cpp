#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/robots/universal_ur10e.hpp>


using namespace jacobi;
using namespace jacobi::robots;

int main() {
    // 1. Set up the robot, transformations, and its kinematics limits
    auto robot = std::make_shared<UniversalUR10e>();
    robot->set_base(Frame::z(0.3));  // [m]
    robot->set_flange_to_tcp(Frame::z(0.15));  // [m]
    robot->set_speed(0.1);  // relative to max speed

    // 2. Setup obstacles in the robot's environment
    auto environment = std::make_shared<Environment>(robot);
    environment->add_obstacle(
        "table",
        Box(1.2, 2.0, 0.3),  // size in [m]
        Frame::from_translation(0.4, 0.0, 0.15)  // origin in [m]
    );
    environment->add_obstacle(
        "divider",
        Box(0.5, 0.1, 0.8),  // size in [m]
        Frame::from_translation(0.74, 0.0, 0.5)  // origin in [m]
    );

    // 3. Set up the planner with the environment
    auto planner = std::make_shared<Planner>(environment);

    // [Cloud version] Authenticate with your account API key by setting
    // the `JACOBI_API_KEY` environment variable.

    // [On-prem version] You can optionally accelerate the computation to milliseconds by loading a trained motion plan
    // planner.load_motion_plan('quick-start.jacobi-plan');

    // 4. Define some start and goal positions and plan the motion
    // 4.1. A simple, collision-free motion can be calculated instantly
    // const auto trajectory = planner.plan(
    //     {-0.86, -1.26, 0.98, -1.20, -1.73, 0.0},  // start in [rad]
    //     {-0.80, -0.73, 1.58, -2.26, -1.42, 0.0}  // goal in [rad]
    // );

    // 4.2. A more complex motion around the 'divider' obstacle
    const auto trajectory = planner->plan(
        {-0.80, -0.73, 1.58, -2.26, -1.42, 0.0},  // start in [rad]
        {0.45, -0.73, 1.40, -2.20, -1.60, 0.5}  // goal in [rad]
    );

    // 5. Print returned trajectory
    if (!trajectory) {
        std::cout << "Could not find a trajectory!" << std::endl;
        return -1;
    }

    std::cout << "Trajectory duration: " << trajectory->duration << " [s]" << std::endl;
    std::cout << "Calculation duration: " << planner->last_calculation_duration << " [ms]" << std::endl;
}
