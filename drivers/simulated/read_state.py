from jacobi import Planner
from jacobi.robots import UniversalUR5e
from jacobi.drivers import SimulatedDriver


if __name__ == '__main__':
    robot = UniversalUR5e()
    planner = Planner(robot)

    driver = SimulatedDriver(planner, sync_with_studio=True)

    print('Joint position:', driver.current_state.position)
