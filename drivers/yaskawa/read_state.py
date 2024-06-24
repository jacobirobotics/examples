from jacobi import Planner
from jacobi.drivers import YaskawaDriver
from jacobi.robots import YaskawaHC10


if __name__ == '__main__':
    robot = YaskawaHC10()
    planner = Planner(robot, 0.025)

    driver = YaskawaDriver(planner)

    print('Joint position:', driver.current_state.position)
