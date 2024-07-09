#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/robots/yaskawa_gp12.hpp>
#include <jacobi/studio/studio.hpp>


using namespace jacobi;
using namespace jacobi::robots;

int main() {
    // 1. Set up the robot, transformations, and its kinematics limits
    auto robot = std::make_shared<YaskawaGP12>();
    robot->base() = Frame::from_euler(0, 0, 0.44, 0, 0, 1.571);
    robot->max_acceleration = {2.0, 2.0, 2.0, 5.0, 5.0, 5.0};

    // 2. Set up the planner without any obstacles
    auto planner = std::make_shared<Planner>(robot, 0.01); // [s]

    // [Cloud version] Authenticate with your account API key by setting
    // the `JACOBI_API_KEY` environment variable.

    // 3. Define start and goal positions
    LinearMotion motion {
        Frame::from_euler(-0.5, 0.9, 0.3, 3.14, 0, 0), // start
        Frame::from_euler(-0.5, 0.9, 1.3, 3.14, 0, 0), // goal
    };

    // 4. Plan the linear motion
    const auto trajectory = planner->plan(motion);
    if (!trajectory) {
        std::cout << "Could not find a trajectory!" << std::endl;
        return -1;
    }

    // 5. Visualize the trajectory in Jacobi Studio
    Studio studio;
    studio.run_trajectory(trajectory.value());
}
