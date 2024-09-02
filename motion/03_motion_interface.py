from pathlib import Path

from jacobi import Convex, Environment, Frame, Motion, Planner, Region, Waypoint
from jacobi.robots import YaskawaGP12


if __name__ == '__main__':
    project_path = Path().parent.absolute() / 'library' / 'demo'

    # 1. Set up the robot, transformations, and its kinematics limits
    robot = YaskawaGP12()
    robot.base = Frame(z=0.765)
    robot.flange_to_tcp = Frame(z=0.2)
    robot.end_effector_obstacle = Convex.load_from_file(project_path / 'ee_hull.obj')
    robot.max_velocity = [4.3, 3.4, 4.3, 7.0, 7.0, 9.0]
    robot.max_acceleration = [7.0, 7.0, 7.5, 15.0, 15.0, 18.0]
    robot.max_jerk = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]

    # 2. Load the obstacle environment from files
    environment = Environment(robot)
    for f in project_path.glob('env/*.obj'):
        environment.add_obstacle(Convex.load_from_file(f))

    # 3. Set up the planner with the cycle time of the robot (alias the timer interval of trajectory steps)
    planner = Planner(robot, delta_time=0.01)  # [s]

    # 4. Define motions that we want to compute
    home = Waypoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [rad]
    buffer = Waypoint([-0.467, 0.2692, -0.3035, -0.1806, 0.0021, 0.0])  # [rad]
    bin_region = Region(min_position=[-2.0, 1.0, -0.7, -1.6, 0.0, -3.2], max_position=[-1.0, 1.5, 0.1, 1.6, 0.7, 3.2])  # [rad]

    home_to_bin = Motion('home-to-bin', robot, home, bin_region)

    # Change some robot parameters for that motion (note that the robot is copied)
    bin_to_buffer = Motion('bin-to-buffer', robot, bin_region, buffer)
    bin_to_buffer.robot.max_acceleration = [5.0, 5.0, 6.0, 10.0, 10.0, 12.0]
    bin_to_buffer.robot.max_jerk = [120.0, 120.0, 120.0, 120.0, 120.0, 120.0]

    buffer_to_home = Motion('buffer-to-home', robot, buffer, home)

    # 5. Add motions to planner
    planner.add_motion(home_to_bin)
    planner.add_motion(bin_to_buffer)
    planner.add_motion(buffer_to_home)

    # 6. Define some dynamic waypoints that are known during runtime
    grasp_joint_positions = [
        Waypoint([-1.3, 1.2, 0.0, -0.9, 0.2, -0.5]),
        Waypoint([-1.4, 1.2, -0.6, -0.9, 0.4, 0.1]),
        Waypoint([-1.5, 1.4, -0.6, -0.9, 0.5, 0.2]),
    ]

    # 7. Compute and return trajectories
    for joint_position in grasp_joint_positions:
        trajectory_home_to_bin = planner.plan('home-to-bin', start=None, goal=joint_position)
        trajectory_bin_to_buffer = planner.plan('bin-to-buffer', start=joint_position, goal=None)
        trajectory_buffer_to_home = planner.plan('buffer-to-home')

        if not trajectory_home_to_bin or not trajectory_bin_to_buffer or not trajectory_buffer_to_home:
            print('Could not find a trajectory!')
            exit()

        cycle_duration = trajectory_home_to_bin.duration + trajectory_bin_to_buffer.duration + trajectory_buffer_to_home.duration
        print(f'Cycle duration: {cycle_duration:0.3f} [s]')
