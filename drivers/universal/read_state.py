from jacobi import Planner
from jacobi.drivers import UniversalDriver
from jacobi.robots import UniversalUR10e


if __name__ == '__main__':
    robot = UniversalUR10e()
    planner = Planner(robot)

    driver = UniversalDriver(planner, '192.168.102.132')

    print('Joint position:', driver.current_state.position)
