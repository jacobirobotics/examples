#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/drivers/fanuc.hpp>
#include <jacobi/robots/fanuc_m_20id25.hpp>
#include <jacobi/utils/vector.hpp>  // E.g. for join method to print vectors easily


using namespace jacobi;
using namespace jacobi::drivers;
using namespace jacobi::robots;


int main() {
    auto robot = std::make_shared<FanucM20iD25>();
    auto planner = std::make_shared<jacobi::Planner>(robot);

    auto driver = std::make_shared<FanucDriver>(planner, "192.168.125.100");

    const auto joint_position = driver->get_current_joint_position();
    std::cout << "Joint position: " << join(joint_position) << std::endl;
}
