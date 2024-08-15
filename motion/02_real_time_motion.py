from jacobi import Frame, Planner
from jacobi.robots import YaskawaGP12


if __name__ == '__main__':
    # 1. Set up the robot, transformations, and its kinematics limits
    robot = YaskawaGP12()
    robot.base = Frame(z=0.765)
    robot.flange_to_tcp = Frame(z=0.2)
    robot.max_velocity = [4.3, 3.4, 4.3, 7.0, 7.0, 9.0]
    robot.max_acceleration = [7.0, 7.0, 7.5, 15.0, 15.0, 18.0]
    robot.max_jerk = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]

    # 2. Set up the planner with the cycle time of the robot (alias the timer interval of trajectory steps)
    #    Here, we skip the environment and all collision checking, so all motion calculations are real-time
    #    capable and take usually less than 0.1ms.
    planner = Planner(robot, delta_time=0.01)  # [s]

    # 3. Define start and goal positions and plan the motion
    home_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    buffer_joint_position = [-0.467, 0.2692, -0.3035, -0.1806, 0.0021, 0.0]

    trajectory = planner.plan(home_joint_position, buffer_joint_position)

    # 4. Print returned trajectory
    if not trajectory:
        print('Could not find a trajectory!')
        exit()

    print(f'Trajectory duration: {trajectory.duration:0.3f} [s]')
    print(f'Calculation duration: {planner.last_calculation_duration:0.2f} [ms]')
