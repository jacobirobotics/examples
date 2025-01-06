#include <iostream>
#include <thread>

#include <jacobi/planner.hpp>
#include <jacobi/drivers/simulated.hpp>
#include <jacobi/robots/universal_ur10e.hpp>
#include <jacobi/utils/vector.hpp>  // E.g. for join method to print vectors easily


using namespace jacobi;
using namespace jacobi::drivers;
using namespace jacobi::robots;


int main() {
    auto robot = std::make_shared<UniversalUR10e>();
    robot->set_speed(0.1);

    auto planner = std::make_shared<Planner>(robot);

    auto driver = std::make_shared<SimulatedDriver>(planner, true);

    // Option A: Use Home position
    Config goal_pose0 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // // Option B: Use current position
    // auto goal_pose0 = driver.get_current_joint_position();

    Config goal_pose1(goal_pose0);
    goal_pose1[0] += 0.2;
    goal_pose1[5] += 0.2;

    Config goal_pose2(goal_pose1);
    goal_pose2[1] += 0.4;
    goal_pose2[2] -= 0.2;
    goal_pose2[3] -= 0.2;

    // Move to: Pose0
    auto result0 = driver->move_to(goal_pose0);
    std::cout << "Move Pose0 result: " << result0.get_description() << std::endl;

    // Move to: Pose1
    const auto result1 = driver->move_to(goal_pose1);
    std::cout << "Move Pose1 result: " << result1.get_description() << std::endl;

    // Move asynchronously to: Pose2
    const auto future_result2 = driver->move_to_async(goal_pose2);

    // Abort with Stop
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto result_stop = driver->stop();
    std::cout << "Stop robot: " << result_stop.get_description() << std::endl;
    std::cout << "Robot position after stop: " << join(driver->get_current_joint_position()) << std::endl;

    // Move to: Pose2 - Get Result
    const auto result2 = future_result2.get();
    std::cout << "Move Pose2 result: " << result2.get_description() << std::endl;

    // Move to: Pose0
    const auto result3 = driver->move_to(goal_pose0);
    std::cout << "Move Pose0 result: " << result3.get_description() << std::endl;
}
