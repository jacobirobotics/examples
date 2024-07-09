from jacobi import Box, Environment, Frame, Planner
from jacobi.robots import UniversalUR10e


if __name__ == '__main__':
    # 1. Set up the robot, transformations, and its kinematics limits
    robot = UniversalUR10e()
    robot.base = Frame(z=0.3)  # [m]
    robot.flange_to_tcp = Frame(z=0.15)  # [m]
    robot.max_acceleration = [4.0, 4.0, 4.0, 4.0, 4.0, 4.0]  # [rad/s^2]

    # 2. Setup obstacles in the robot's environment
    environment = Environment(robot)
    environment.add_obstacle(
        name='table',
        object=Box(1.2, 2.0, 0.3),  # size in [m]
        origin=Frame.from_translation(0.4, 0.0, 0.15),  # origin in [m]
    )
    environment.add_obstacle(
        name='divider',
        object=Box(0.5, 0.1, 0.8),  # size in [m]
        origin=Frame.from_translation(0.74, 0.0, 0.5),  # origin in [m]
    )

    # 3. Set up the planner with the cycle time of the robot (alias the timer interval of trajectory steps)
    planner = Planner(environment, 0.004)  # delta time in [s]

    # [Cloud version] Authenticate with your account API key by setting
    # the `JACOBI_API_KEY` environment variable.

    # [On-prem version] You can optionally accelerate the computation to milliseconds by loading a trained motion plan
    # planner.load_motion_plan('quick-start.jacobi-plan')

    # 4. Define some start and goal positions and plan the motion
    # 4.1. A simple, collision-free motion can be calculated instantly
    # trajectory = planner.plan(
    #     start=[-0.86, -1.26, 0.98, -1.20, -1.73, 0.0],
    #     goal=[-0.80, -0.73, 1.58, -2.26, -1.42, 0.0],
    # )

    # 4.2. A more complex motion around the 'divider' obstacle
    trajectory = planner.plan(
        start=[-0.80, -0.73, 1.58, -2.26, -1.42, 0.0],
        goal=[0.45, -0.73, 1.40, -2.20, -1.60, 0.5],
    )

    # 5. Print returned trajectory
    if not trajectory:
        print('Could not find a trajectory!')
        exit()

    print(f'Trajectory duration: {trajectory.duration:0.3f} [s]')
    print(f'Calculation duration: {planner.last_calculation_duration:0.2f} [ms]')
