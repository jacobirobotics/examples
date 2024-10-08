#include <iostream>

#include <jacobi/planner.hpp>


using namespace jacobi;

int main() {
    // 1. Set up planner by passing the project name in Jacobi Studio
    auto planner = Planner::load_from_studio("My Project");

    // 2. Plan motion defined in Studio project
    const auto trajectory = planner->plan("Home to Camera");

    // 3. Plan with new start and goal positions
    // const auto trajectory = planner.plan({1.6, 0.4, 0.1, 0, 0.5, -1.5}, {0.0, 0.4, 0.6, 0.0, 0.4, -0.4});

    // 4. Print returned trajectory
    if (!trajectory) {
        std::cout << "Could not find a trajectory!" << std::endl;
        return -1;
    }

    std::cout << "Trajectory duration: " << trajectory->duration << " [s]" << std::endl;
    std::cout << "Calculation duration: " << planner->last_calculation_duration << " [ms]" << std::endl;
}
