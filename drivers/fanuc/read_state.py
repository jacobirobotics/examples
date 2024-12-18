from jacobi import Planner
from jacobi.robots import FanucM20iD25
from jacobi.drivers import FanucDriver


if __name__ == '__main__':
    robot = FanucM20iD25()
    planner = Planner(robot)

    driver = FanucDriver(planner, host='192.168.125.100')

    print('Joint position:', driver.current_joint_position)
