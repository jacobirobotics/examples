from jacobi import Frame, LinearMotion, Planner, Studio
from jacobi.robots import YaskawaGP12


if __name__ == '__main__':
    # 1. Set up the robot, transformations, and its kinematics limits
    robot = YaskawaGP12()
    robot.base = Frame.from_euler(0, 0, 0.44, 0, 0, 1.571)
    robot.max_acceleration = [2.0, 2.0, 2.0, 5.0, 5.0, 5.0]

    # 2. Set up the planner without any obstacles
    planner = Planner(robot, delta_time=0.01)  # [s]

    # [Cloud version] Authenticate with your account API key by setting
    # the `JACOBI_API_KEY` environment variable.

    # 3. Define start and goal positions
    motion = LinearMotion(
        start=Frame.from_euler(-0.5, 0.9, 0.3, 3.14, 0, 0),
        goal=Frame.from_euler(-0.5, 0.9, 1.3, 3.14, 0, 0),
    )

    # 4. Plan the linear motion
    trajectory = planner.plan(motion)
    if not trajectory:
        raise RuntimeError('Could not calculate a valid trajectory.')

    # 5. Visualize the trajectory in Jacobi Studio
    studio = Studio()
    studio.speedup = 5.0
    studio.run_trajectory(trajectory, loop_forever=True)
