#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/robots/abb_irb1200_5_90.hpp>
#include <jacobi/drivers/abb.hpp>
#include <jacobi/utils/vector.hpp>  // E.g. for join method to print vectors easily


using namespace jacobi;
using namespace jacobi::drivers;
using namespace jacobi::robots;


int main() {
    auto robot = std::make_shared<ABBIRB1200590>();
    auto planner = std::make_shared<Planner>(robot);

    auto driver = std::make_shared<ABBDriver>(planner, "192.168.125.1", 6511);

    const auto joint_state = driver->get_current_state();
    std::cout << "Joint position: " << join(joint_state.position) << std::endl;
}
