#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/drivers/yaskawa.hpp>
#include <jacobi/robots/yaskawa_hc10.hpp>


using namespace jacobi;
using namespace jacobi::drivers;
using namespace jacobi::robots;


int main() {
    auto robot = std::make_shared<YaskawaHC10>();
    auto planner = std::make_shared<Planner>(robot, 0.025);

    auto driver = std::make_shared<YaskawaDriver>(planner);

    driver->create_io_alias("sensor", 10);
    driver->create_io_alias("gripper", 1003);

    // Read the value of the digital input
    const auto value = driver->get_digital_input("sensor");
    if (!value) {
        std::cout << "Could not read from address 'sensor'." << std::endl;
        std::exit(1);
    }
    std::cout << "Value of digital input 'sensor' is '" << value.value() << "'." << std::endl;

    // Set an output for a group of 8 bits
    const bool success = driver->set_group_output("gripper", 1);
    if (!success) {
        std::cout << "Could not write to address 'gripper'." << std::endl;
        std::exit(1);
    }
}