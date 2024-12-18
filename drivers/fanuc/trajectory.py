from argparse import ArgumentParser

from jacobi import Planner, Trajectory
from jacobi.drivers import FanucDriver
from jacobi.robots import FanucM20iD25


if __name__ == '__main__':
    parser = ArgumentParser('fanuc-driver', description='Run a pre-calculated trajectory on a FANUC robot.')
    parser.add_argument('trajectory', type=str, help='File of the trajectory.')
    parser.add_argument('--speed', default=1.0, type=float, help='Speed of the driver.')
    parser.add_argument('--host', default='192.168.0.130', help='IP address of the robot.')
    parser.add_argument('--move-to-start', action='store_true', help='Move to the start of the trajectory.')

    args = parser.parse_args()

    robot = FanucM20iD25()
    planner = Planner(robot)

    driver = FanucDriver(planner, host=args.host)
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
