from jacobi import Frame, Planner, PathFollowingMotion, LinearPath, CircularPath, BlendedPath
from jacobi.robots import UniversalUR10e
import numpy as np


if __name__ == '__main__':
    # Set up the robot and planner
    robot = UniversalUR10e()
    robot.max_acceleration = [4.0, 4.0, 4.0, 4.0, 4.0, 4.0]  # [rad/s^2]

    planner = Planner(robot)

    # The velocity we want to follow the path with
    velocity = 0.8  # [m/s]

    # 1. Follow the linear trajectory with constant orientation
    print('\nLinear trajectory with constant orientation')
    start = Frame.from_translation(0.6, 0.3, 0.1)
    goal = Frame.from_translation(0.6, -0.3, 0.1)
    motion = PathFollowingMotion(robot, LinearPath(start, goal), velocity)
    traj = planner.plan(motion)
    print(f'Trajectory duration: {traj.duration} [s]')

    # 2. Follow the linear trajectory with orientation interpolation
    print('\nLinear trajectory with orientation interpolation')
    goal = Frame.from_euler(0.6, -0.3, 0.1, 0.0, 1.571, 0.0)
    motion = PathFollowingMotion(robot, LinearPath(start, goal), velocity)
    traj = planner.plan(motion)
    print(f'Trajectory duration: {traj.duration} [s]')

    # 3. Follow the circular trajectory with constant global orientation
    print('\nCircular trajectory with constant global orientation')
    start = Frame.from_euler(0.7, 0.8, 0.3, 0.0, 0.0, 0.0)
    center = np.array([0.7, 0.7, 0.6])
    angle = 3.0
    normal = np.array([1.0, 0.0, 0.0])
    motion = PathFollowingMotion(robot, CircularPath(start, angle, center, normal), velocity)
    traj = planner.plan(motion)
    print(f'Trajectory duration: {traj.duration} [s]')

    # 4. Follow the circular trajectory with constant tool-to-surface orientation
    print('\nCircular trajectory with constant tool-to-surface orientation')
    motion = PathFollowingMotion(robot, CircularPath(start, angle, center, normal, True), velocity)
    traj = planner.plan(motion)
    print(f'Trajectory duration: {traj.duration} [s]')

    # 5. Follow the blended trajectory with constant global orientation
    print('\nBlended trajectory with constant global orientation')
    waypoints = [
        Frame.from_translation(0.3, 0.3, -0.3),
        Frame.from_translation(0.3, 0.3, 0.1),
        Frame.from_translation(0.3, -0.3, 0.1),
        Frame.from_translation(0.3, -0.3, -0.3),
        Frame.from_translation(0.3, 0.3, -0.3),
    ]

    motion = PathFollowingMotion(robot, BlendedPath(waypoints, 0.1), velocity)
    traj = planner.plan(motion)
    print(f'Trajectory duration: {traj.duration} [s]')

    # 6. Follow the blended trajectory with constant tool-to-surface orientation
    print('\nBlended trajectory with constant tool-to-surface orientation')
    waypoints = [
        Frame.from_euler(0.3, 0.3, -0.3, 1.57, 0.0, 0.0),
        Frame.from_translation(0.3, 0.3, 0.1),
        Frame.from_translation(0.3, -0.3, 0.1),
        Frame.from_translation(0.3, -0.3, -0.3),
        Frame.from_translation(0.3, 0.3, -0.3),
    ]

    motion = PathFollowingMotion(robot, BlendedPath(waypoints, 0.1, True), velocity)
    traj = planner.plan(motion)
    print(f'Trajectory duration: {traj.duration} [s]')

    # 7. Follow the blended trajectory without a predefined blend radius
    print('\nBlended trajectory without a predefined blend radius')
    motion = PathFollowingMotion(robot, BlendedPath(waypoints, True), velocity)
    traj = planner.plan(motion)
    print(f'Trajectory duration: {traj.duration} [s]')
