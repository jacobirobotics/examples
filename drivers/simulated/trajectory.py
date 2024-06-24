from argparse import ArgumentParser

from jacobi import Planner, Trajectory
from jacobi.drivers import SimulatedDriver
from jacobi.robots import UniversalUR10e


if __name__ == '__main__':
    parser = ArgumentParser('simulated-driver', description='Run a pre-calculated trajectory on a simulated robot.')
    parser.add_argument('trajectory', type=str, help='File of the trajectory.')
    parser.add_argument('--speed', default=1.0, type=float, help='Speed of the driver.')
    parser.add_argument('--without-studio', action='store_true', help="Don't sync with Studio Live.")
    parser.add_argument('--move-to-start', action='store_true', help='Move to the start of the trajectory.')

    args = parser.parse_args()

    robot = UniversalUR10e()
    planner = Planner(robot)

    driver = SimulatedDriver(planner, sync_with_studio=not args.without_studio)
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
