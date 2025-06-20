from jacobi import Planner, LowLevelMotion
from jacobi.robots import FrankaPanda


if __name__ == '__main__':
    # 1. Set up the robot, transformations, and its kinematics limits
    robot = FrankaPanda()
    robot.set_speed(0.2)

    # 2. Set up the planner
    planner = Planner(robot)

    motion = LowLevelMotion()
    motion.start = [-1.27, 0.61, -0.46, -1.57, -1.81, 1.0, 0.0]
    motion.intermediate_positions = [
        [-1.38, 0.64, -0.46, -1.57, -1.82, 1.0, 0.0],
        [-1.49, 0.69, -0.46, -1.57, -1.81, 1.1, 0.0],
        [-1.52, 0.72, -0.46, -1.57, -1.72, 1.3, 0.0],
        [-1.66, 0.75, -0.46, -1.57, -1.50, 1.6, 0.0],
    ]
    motion.goal = [-1.69, 0.76, -0.46, -1.57, -1.36, 1.7, 0.0]

    # 3. Plan the low-level motion
    trajectory = planner.plan(motion)

    # 4. Print returned trajectory
    if not trajectory:
        print('Could not find a trajectory!')
        exit()

    print(f'Trajectory duration: {trajectory.duration:0.3f} [s]')
    print(f'Calculation duration: {planner.last_calculation_duration:0.2f} [ms]')
