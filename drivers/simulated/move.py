import time

from jacobi import Planner
from jacobi.drivers import SimulatedDriver
from jacobi.robots import UniversalUR10e


if __name__ == '__main__':
    robot = UniversalUR10e()
    robot.set_speed(0.1)

    planner = Planner(robot)

    driver = SimulatedDriver(planner, sync_with_studio=True)

    # Option A: Use Home position
    target_pose0 = [-1.6007, -1.7271, -2.203, -0.808, 1.5951, -0.031]
    # # Option B: Use current position
    # target_pose0 = driver.current_joint_position
    print('Start robot position:', target_pose0)

    target_pose1 = list(target_pose0)
    target_pose1[0] += 0.2
    target_pose1[5] += 0.2

    target_pose2 = list(target_pose1)
    target_pose2[1] += 0.4
    target_pose2[2] -= 0.2
    target_pose2[3] -= 0.2

    # Move to: Pose0
    print('Target robot position - Pose0: ', target_pose0)
    result0 = driver.move_to(target_pose0)
    print('Move Pose0 result:', result0)
    print('Current robot position:', driver.current_joint_position)

    # Move to: Pose1
    print('Target robot position - Pose1: ', target_pose1)
    result1 = driver.move_to(target_pose1)
    print('Move Pose1 result:', result1)
    print('Current robot position:', driver.current_joint_position)

    # Move asynchronously to: Pose2
    print('Target robot position - Pose2: ', target_pose2)
    future_result2 = driver.move_to_async(target_pose2)

    # Abort with Stop
    time.sleep(1)
    result_stop = driver.stop()
    print('Stop robot:', result_stop)
    print('Robot position after stop:', driver.current_joint_position)

    # Move to: Pose2 - Get Result
    result2 = future_result2.get()
    print(f'Move Pose2 result: {result2}')

    # Move to: Pose0
    print('Target robot position - Pose0: ', target_pose0)
    result3 = driver.move_to(target_pose0)
    print('Move Pose0 result:', result3)
    print('Current robot position:', driver.current_joint_position)
