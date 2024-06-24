#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/drivers/universal.hpp>
#include <jacobi/robots/universal_ur10e.hpp>
#include <jacobi/utils/vector.hpp>  // E.g. for join method to print vectors easily


using namespace jacobi;
using namespace jacobi::drivers;
using namespace jacobi::robots;


int main() {
    auto robot = std::make_shared<UniversalUR10e>();
    auto planner = std::make_shared<Planner>(robot);

    auto driver = std::make_shared<UniversalDriver>(planner, "192.168.102.132");

    const auto joint_state = driver->get_current_state();
    std::cout << "Joint position: " << join(joint_state.position) << std::endl;
}
