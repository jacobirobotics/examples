#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/drivers/yaskawa.hpp>
#include <jacobi/robots/yaskawa_hc10.hpp>
#include <jacobi/utils/vector.hpp>  // E.g. for join method to print vectors easily


using namespace jacobi;
using namespace jacobi::drivers;
using namespace jacobi::robots;


int main() {
    auto robot = std::make_shared<YaskawaHC10>();
    auto planner = std::make_shared<Planner>(robot, 0.025);

    auto driver = std::make_shared<YaskawaDriver>(planner);

    const auto joint_state = driver->get_current_state();
    std::cout << "Joint position: " << join(joint_state.position) << std::endl;
}
