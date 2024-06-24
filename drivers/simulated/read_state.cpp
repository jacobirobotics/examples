#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/robots/universal_ur5e.hpp>
#include <jacobi/drivers/simulated.hpp>
#include <jacobi/utils/vector.hpp>  // E.g. for join method to print vectors easily


using namespace jacobi;
using namespace jacobi::drivers;
using namespace jacobi::robots;


int main() {
    auto robot = std::make_shared<UniversalUR5e>();
    auto planner = std::make_shared<Planner>(robot);

    auto driver = std::make_shared<SimulatedDriver>(planner, true);

    const auto joint_state = driver->get_current_state();
    std::cout << "Joint position: " << join(joint_state.position) << std::endl;
}
