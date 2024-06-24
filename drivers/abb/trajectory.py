from argparse import ArgumentParser

from jacobi import Planner, Trajectory
from jacobi.drivers import ABBDriver
from jacobi.robots import ABBIRB1200590


if __name__ == '__main__':
    parser = ArgumentParser('abb-driver', description='Run a pre-calculated trajectory on an ABB robot.')
    parser.add_argument('trajectory', type=str, help='File of the trajectory.')
    parser.add_argument('--speed', default=1.0, type=float, help='Speed of the driver.')
    parser.add_argument('--host', default='192.168.125.1', help='IP address of the robot.')
    parser.add_argument('--port', default=6511, type=int, help='Port of the EGM Server.')
    parser.add_argument('--move-to-start', action='store_true', help='Move to the start of the trajectory.')

    args = parser.parse_args()

    robot = ABBIRB1200590()
    planner = Planner(robot)

    driver = ABBDriver(planner, host=args.host, port=args.port)
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
