from jacobi import Planner
from jacobi.robots import ABBIRB1200590
from jacobi.drivers import ABBDriver


if __name__ == '__main__':
    robot = ABBIRB1200590()
    planner = Planner(robot)

    driver = ABBDriver(planner, host='192.168.125.1', port=6511)

    print('Joint position:', driver.current_state.position)
