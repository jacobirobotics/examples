#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/robots/franka_panda.hpp>


using namespace jacobi;
using namespace jacobi::robots;

int main() {
    // 1. Set up the robot and its kinematics limits
    auto robot = std::make_shared<FrankaPanda>();
    robot->set_speed(0.2);

    // 2. Set up the planner
    auto planner = std::make_shared<Planner>(robot);

    // [Cloud version] Authenticate with your account API key by setting
    // the `JACOBI_API_KEY` environment variable.

    LowLevelMotion motion;
    motion.start = {-1.27, 0.61, -0.46, 0.0, -1.81, 0.0, 0.0};
    motion.intermediate_positions = {
        {-1.38, 0.64, -0.46, 0.0, -1.82, 0.0, 0.0},
        {-1.49, 0.69, -0.46, 0.0, -1.81, 0.1, 0.0},
        {-1.52, 0.72, -0.46, 0.0, -1.72, 0.3, 0.0},
        {-1.66, 0.75, -0.46, 0.0, -1.50, 0.6, 0.0},
    };
    motion.goal = {-1.69, 0.76, -0.46, 0.0, -1.36, 0.7, 0.0};

    // 3. Plan the low-level motion
    const auto trajectory = planner->plan(motion);

    // 4. Print returned trajectory
    if (!trajectory) {
        std::cout << "Could not find a trajectory!" << std::endl;
        return -1;
    }

    std::cout << "Trajectory duration: " << trajectory->duration << " [s]" << std::endl;
    std::cout << "Calculation duration: " << planner->last_calculation_duration << " [ms]" << std::endl;
}
