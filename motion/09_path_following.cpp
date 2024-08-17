#include <iostream>

#include <jacobi/motions/path_following_motion.hpp>
#include <jacobi/planner.hpp>
#include <jacobi/robots/universal_ur10e.hpp>

using namespace jacobi;
using namespace jacobi::robots;


int main() {
    // Set up the robot and planner
    auto robot = std::make_shared<UniversalUR10e>();
    robot->max_acceleration = {4.0, 4.0, 4.0, 4.0, 4.0, 4.0};  // [rad/s^2]

    auto planner = std::make_shared<Planner>(robot);

    // The velocity we want to follow the path with
    const double velocity = 0.8; // [m/s]

    // 1. Follow the linear trajectory with constant orientation
    std::cout << "\nLinear trajectory with constant orientation" << std::endl;
    auto start = Frame::from_translation(0.6, 0.3, 0.1);
    auto goal = Frame::from_translation(0.6, -0.3, 0.1);
    auto motion = PathFollowingMotion(robot, std::make_shared<LinearPath>(start, goal), velocity);
    auto traj = planner->plan(motion);
    std::cout << "Trajectory duration: " << traj->duration << " [s]" << std::endl;

    // 2. Follow the linear trajectory with orientation interpolation
    std::cout << "\nLinear trajectory with orientation interpolation" << std::endl;
    goal = Frame::from_euler(0.6, -0.3, 0.1, 0.0, 1.571, 0.0);
    motion = PathFollowingMotion(robot, std::make_shared<LinearPath>(start, goal), velocity);
    traj = planner->plan(motion);
    std::cout << "Trajectory duration: " << traj->duration << " [s]" << std::endl;

    // 3. Follow the circular trajectory with constant global orientation
    std::cout << "\nCircular trajectory with constant global orientation" << std::endl;
    start = Frame::from_euler(0.7, 0.8, 0.3, 0.0, 0.0, 0.0);
    const std::vector<double> center = {0.7, 0.7, 0.6};
    const float angle = 3.0;
    const std::vector<double> normal = {1.0, 0.0, 0.0};
    motion = PathFollowingMotion(robot, std::make_shared<CircularPath>(start, angle, center, normal, false), velocity);
    traj = planner->plan(motion);
    std::cout << "Trajectory duration: " << traj->duration << " [s]" << std::endl;

    // 4. Follow the circular trajectory with constant tool-to-surface orientation
    std::cout << "\nCircular trajectory with constant tool-to-surface orientation" << std::endl;
    motion = PathFollowingMotion(robot, std::make_shared<CircularPath>(start, angle, center, normal, true), velocity);
    traj = planner->plan(motion);
    std::cout << "Trajectory duration: " << traj->duration << " [s]" << std::endl;

    // 5. Follow the waypoints with linear segments and circular blends and constant global orientation
    std::cout << "\nBlended trajectory with constant global orientation" << std::endl;
    std::vector<Frame> waypoints = {
        Frame::from_translation(0.3, 0.3, -0.3),
        Frame::from_translation(0.3, 0.3, 0.1),
        Frame::from_translation(0.3, -0.3, 0.1),
        Frame::from_translation(0.3, -0.3, -0.3),
        Frame::from_translation(0.3, 0.3, -0.3),
    };
    motion = PathFollowingMotion(robot, std::make_shared<BlendedPath>(waypoints, 0.1, false), velocity);
    traj = planner->plan(motion);
    std::cout << "Trajectory duration: " << traj->duration << " [s]" << std::endl;

    // 6. Follow the waypoints with linear segments and circular blends and constant tool-to-surface orientation
    std::cout << "\nBlended trajectory with constant tool-to-surface orientation" << std::endl;
    waypoints = {
        Frame::from_euler(0.3, 0.3, -0.3, 1.57, 0.0, 0.0),
        Frame::from_translation(0.3, 0.3, 0.1),
        Frame::from_translation(0.3, -0.3, 0.1),
        Frame::from_translation(0.3, -0.3, -0.3),
        Frame::from_translation(0.3, 0.3, -0.3),
    };
    motion = PathFollowingMotion(robot, std::make_shared<BlendedPath>(waypoints, 0.1, true), velocity);
    traj = planner->plan(motion);
    std::cout << "Trajectory duration: " << traj->duration << " [s]" << std::endl;

    // 7. Follow the blended trajectory without a predefined blend radius
    std::cout << "\nBlended trajectory without a predefined blend radius" << std::endl;
    motion = PathFollowingMotion(robot, std::make_shared<BlendedPath>(waypoints, true), velocity);
    traj = planner->plan(motion);
    std::cout << "Trajectory duration: " << traj->duration << " [s]" << std::endl;
}
