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
    robot->set_speed(0.5);  // You can also set the robot's speed on the teach pendant

    auto planner = std::make_shared<Planner>(robot);

    auto driver = std::make_shared<FanucDriver>(planner, "192.168.20.230");

    driver->enable();

    // Option A: Use Home position
    Config goal_pose0 {0.0, 0.0, 0.0, 0.0, -M_PI_2, 0.0};
    // // Option B: Use current position
    // auto goal_pose0 = driver.get_current_joint_position();

    Config goal_pose1(goal_pose0);
    goal_pose1[0] += 0.2;
    goal_pose1[5] -= 0.2;

    Config goal_pose2(goal_pose1);
    goal_pose2[1] += 0.4;
    goal_pose2[2] += 0.2;
    goal_pose2[4] += 0.2;

    // Move to: Pose0
    const auto result0 = driver->move_to(goal_pose0);
    std::cout << "Move to home result: " << result0.get_description() << std::endl;

    // Move to: Pose1
    const auto result1 = driver->move_to(goal_pose1);
    std::cout << "Move to pose1 result: " << result1.get_description() << std::endl;

    // Move asynchronously to: Pose2
    const auto future_result2 = driver->move_to_async(goal_pose2);

    // Abort with Stop
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    const auto result_stop = driver->stop();
    std::cout << "Stop robot: " << result_stop.get_description() << std::endl;
    std::cout << "Robot position after stop: " << join(driver->get_current_joint_position()) << std::endl;

    // Move to: Pose2 - Get Result
    const auto result2 = future_result2.get();
    std::cout << "Move to pose2 result: " << result2.get_description() << std::endl;

    // Move to: Pose0
    const auto result3 = driver->move_to(goal_pose0);
    std::cout << "Move to home result: " << result3.get_description() << std::endl;

    driver->disable();
}
