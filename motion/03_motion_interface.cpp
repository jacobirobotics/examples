#include <iostream>

#include <jacobi/planner.hpp>
#include <jacobi/robots/yaskawa_gp12.hpp>


using namespace jacobi;
using namespace jacobi::robots;

int main() {
    std::filesystem::path project_path {"../library/demo"};

    // 1. Set up the robot, transformations, and its kinematics limits
    auto robot = std::make_shared<YaskawaGP12>();
    robot->base() = Frame::from_translation(0.0, 0.0, 0.765);
    robot->flange_to_tcp() = Frame::from_translation(0.0, 0.0, 0.2);
    robot->end_effector_obstacle = Convex::load_from_file(project_path / "ee_hull.obj");
    robot->max_velocity = {4.3, 3.4, 4.3, 7.0, 7.0, 9.0};
    robot->max_acceleration = {7.0, 7.0, 7.5, 15.0, 15.0, 18.0};
    robot->max_jerk = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    // 2. Load the obstacle environment from files
    auto environment = std::make_shared<Environment>(robot);
    for (auto file: std::filesystem::directory_iterator(project_path / "env")) {
        if (file.path().extension() != ".obj" || !file.is_regular_file()) {
            continue;
        }

        environment->add_obstacle(Convex::load_from_file(file));
    }

    // 3. Set up the planner with the cycle time of the robot (alias the timer interval of trajectory steps)
    auto planner = std::make_shared<Planner>(environment, 0.01); // [s]

    // [Cloud version] Authenticate with your account API key by setting
    // the `JACOBI_API_KEY` environment variable.

    // 4. Define motions that we want to compute
    Waypoint home {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; // [rad]
    Waypoint buffer {{-0.467, 0.2692, -0.3035, -0.1806, 0.0021, 0.0}}; // [rad]
    Region bin_region {{-2.0, 1.0, -0.7, -1.6, 0.0, -3.2}, {-1.0, 1.5, 0.1, 1.6, 0.7, 3.2}}; // lower bound, upper bound, [rad]

    Motion home_to_bin {"home-to-bin", robot, home, bin_region};

    // Change some robot parameters for that motion (note that the robot is copied)
    Motion bin_to_buffer {"bin-to-buffer", robot, bin_region, buffer};
    bin_to_buffer.robot_arm()->max_acceleration = {5.0, 5.0, 6.0, 10.0, 10.0, 12.0};
    bin_to_buffer.robot_arm()->max_jerk = {120.0, 120.0, 120.0, 120.0, 120.0, 120.0};

    Motion buffer_to_home {"buffer-to-home", robot, buffer, home};

    // 5. Add motions to planner
    planner->add_motion(home_to_bin);
    planner->add_motion(bin_to_buffer);
    planner->add_motion(buffer_to_home);

    // 6. Define some dynamic waypoints that are known during runtime
    std::vector<Waypoint> grasp_joint_positions = {
        {{-1.3, 1.2, 0.0, -0.9, 0.2, -0.5}},
        {{-1.4, 1.2, -0.6, -0.9, 0.4, 0.1}},
        {{-1.5, 1.4, -0.6, -0.9, 0.5, 0.2}}
    };

    // 7. Compute and return trajectories
    for (const auto& joint_position: grasp_joint_positions) {
        const auto trajectory_home_to_bin = planner->plan("home-to-bin", std::nullopt, joint_position);
        const auto trajectory_bin_to_buffer = planner->plan("bin-to-buffer", joint_position, std::nullopt);
        const auto trajectory_buffer_to_home = planner->plan("buffer-to-home");

        if (!trajectory_home_to_bin || !trajectory_bin_to_buffer || !trajectory_buffer_to_home) {
            std::cout << "Could not find a trajectory cycle!" << std::endl;
            continue;
        }
        const double cycle_duration = trajectory_home_to_bin->duration + trajectory_bin_to_buffer->duration + trajectory_buffer_to_home->duration;
        std::cout << "Cycle duration: " << cycle_duration << " [s]" << std::endl;
    }
}
