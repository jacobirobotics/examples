#include <iostream>

#include <cxxopts/cxxopts.hpp>

#include <jacobi/planner.hpp>
#include <jacobi/drivers/yaskawa.hpp>
#include <jacobi/robots/yaskawa_hc10.hpp>


using namespace jacobi;
using namespace jacobi::drivers;
using namespace jacobi::robots;


int main(int argc, char *argv[]) {
    cxxopts::Options options("yaskawa-driver", "Run a pre-calculated trajectory on a Yaskawa robot.");
    options.add_options()
        ("trajectory", "File of the trajectory.", cxxopts::value<std::string>())
        ("speed", "Speed of the driver.", cxxopts::value<double>()->default_value("1.0"))
        ("host", "IP address of the robot.", cxxopts::value<std::string>()->default_value("192.168.1.31"))
        ("move-to-start", "Move to the start of the trajectory.", cxxopts::value<bool>()->default_value("false"))
        ("h,help", "Print usage")
    ;

    options.parse_positional({"trajectory"});

    const auto args = options.parse(argc, argv);
    if (args.count("help")) {
        std::cout << options.help() << std::endl;
        std::exit(0);

    } else if (args.count("trajectory") == 0) {
        std::cout << "Please specifiy a '*.json' trajectory file to run." << std::endl;
        std::exit(1);
    }

    auto robot = std::make_shared<YaskawaHC10>();
    auto planner = std::make_shared<Planner>(robot, 0.025);  // [s]

    auto driver = std::make_shared<YaskawaDriver>(planner, args["host"].as<std::string>());
    driver->set_speed(args["speed"].as<double>());

    const auto trajectory = Trajectory::from_json_file(args["trajectory"].as<std::string>());

    if (args["move-to-start"].as<bool>()) {
        const auto result = driver->move_to(trajectory.positions.front());
        if (!result) {
            std::cout << "Move to start failed with result: " << result.get_description() << std::endl;
            std::exit(1);
        }
    }

    const auto result = driver->run(trajectory);
    if (!result) {
        std::cout << "Failed to run the trajectory: " << result.get_description() << std::endl;
        std::exit(1);
    }
}
