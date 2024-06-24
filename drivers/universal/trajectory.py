from argparse import ArgumentParser

from jacobi import Planner, Trajectory
from jacobi.drivers import UniversalDriver
from jacobi.robots import UniversalUR10e


if __name__ == '__main__':
    parser = ArgumentParser('universal-driver', description='Run a pre-calculated trajectory on a UR robot.')
    parser.add_argument('trajectory', type=str, help='File of the trajectory.')
    parser.add_argument('--speed', default=1.0, type=float, help='Speed of the driver.')
    parser.add_argument('--host', default='192.168.102.132', help='IP address of the robot.')
    parser.add_argument('--move-to-start', action='store_true', help='Move to the start of the trajectory.')

    args = parser.parse_args()

    robot = UniversalUR10e()
    planner = Planner(robot)

    driver = UniversalDriver(planner, host=args.host)
    driver.speed = args.speed

    trajectory = Trajectory.from_json_file(args.trajectory)

    if args.move_to_start:
        result = driver.move_to(trajectory.positions[0])
        if not result:
            print(f'Move to start failed with result: {result}')
            exit()

    result = driver.run(trajectory)
    if not result:
        print(f'Failed to run the trajectory: {result}')
        exit()
