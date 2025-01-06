import time

from jacobi import Planner
from jacobi.robots import ABBIRB1200590
from jacobi.drivers import ABBDriver


if __name__ == '__main__':
    robot = ABBIRB1200590()
    robot.set_speed(0.1)

    planner = Planner(robot)

    driver = ABBDriver(planner, host='192.168.125.1', port=6511)

    # Option A: Use Home position
    goal_pose0 = [0.0, 0.0, 0.0, 0.0, -1.570796, 0.0]
    # # Option B: Use current position
    # goal_pose0 = driver.current_joint_position
    # print('Start robot position:', goal_pose0)

    goal_pose1 = list(goal_pose0)
    goal_pose1[0] += 0.2
    goal_pose1[5] -= 0.2

    goal_pose2 = list(goal_pose1)
    goal_pose2[1] += 0.4
    goal_pose2[2] += 0.2
    goal_pose2[4] += 0.2

    # Move to: Pose0
    result0 = driver.move_to(goal_pose0)
    print('Move Pose0 result:', result0)

    # Move to: Pose1
    result1 = driver.move_to(goal_pose1)
    print('Move Pose1 result:', result1)

    # Move asynchronously to: Pose2
    future_result2 = driver.move_to_async(goal_pose2)

    # Abort with Stop
    time.sleep(1)
    result_stop = driver.stop()
    print('Stop robot:', result_stop)
    print('Robot position after stop:', driver.current_joint_position)

    # Move to: Pose2 - Get Result
    result2 = future_result2.get()
    print(f'Move Pose2 result: {result2}')

    # Move to: Pose0
    result3 = driver.move_to(goal_pose0)
    print(f'Move to home result: {result3}')
