#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/drivers/abb.hpp>
#include <jacobi/robots/abb_irb1200_5_90.hpp>


using namespace jacobi;
using namespace jacobi::drivers;
using namespace jacobi::robots;


int main() {
    auto robot = std::make_shared<ABBIRB1200590>();
    auto planner = std::make_shared<Planner>(robot);

    auto driver = std::make_shared<ABBDriver>(planner, "192.168.125.1", 6511);

    // Read the value of the digital input
    auto value = driver->get_digital_input("gripper");
    if (!value) {
        std::cout << "Could not read from address 'gripper'." << std::endl;
        std::exit(1);
    }
    std::cout << "Value of digital input 'gripper' is '" << value.value() << "'." << std::endl;

    // Set an output for a group of 8 bits
    const bool success = driver->set_group_output("gripper", 1);
    if (!success) {
        std::cout << "Could not write to address 'gripper'." << std::endl;
        std::exit(1);
    }

    // Read value again
    value = driver->get_digital_input("gripper");
    std::cout << "Value of digital input 'gripper' is now '" << value.value() << "'." << std::endl;
}