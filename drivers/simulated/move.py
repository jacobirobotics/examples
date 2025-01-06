from time import sleep

from jacobi import Planner
from jacobi.drivers import SimulatedDriver
from jacobi.robots import UniversalUR10e


if __name__ == '__main__':
    robot = UniversalUR10e()
    robot.set_speed(0.1)

    planner = Planner(robot)

    driver = SimulatedDriver(planner, sync_with_studio=True)

    # Option A: Use Home position
    goal_pose0 = [-1.6007, -1.7271, -2.203, -0.808, 1.5951, -0.031]
    # # Option B: Use current position
    # goal_pose0 = driver.current_joint_position
    # print('Start robot position:', goal_pose0)

    goal_pose1 = list(goal_pose0)
    goal_pose1[0] += 0.2
    goal_pose1[5] += 0.2

    goal_pose2 = list(goal_pose1)
    goal_pose2[1] += 0.4
    goal_pose2[2] -= 0.2
    goal_pose2[3] -= 0.2

    # Move to: Pose0
    result0 = driver.move_to(goal_pose0)
    print(f'Move to home result: {result0}')

    # Move to: Pose1
    result1 = driver.move_to(goal_pose1)
    print(f'Move to pose1 result: {result1}')

    # Move asynchronously to: Pose2
    future_result2 = driver.move_to_async(goal_pose2)

    # Abort with Stop
    sleep(1)
    result_stop = driver.stop()
    print(f'Stop robot: {result_stop}')
    print(f'Robot position after stop: {driver.current_joint_position}')

    # Move to: Pose2 - Get Result
    result2 = future_result2.get()
    print(f'Move to pose2 result: {result2}')

    # Move to: Pose0
    result3 = driver.move_to(goal_pose0)
    print(f'Move to home result: {result3}')
