from jacobi import Planner
from jacobi.drivers import FanucDriver
from jacobi.robots import FanucM20iD25


if __name__ == '__main__':
    robot = FanucM20iD25()
    planner = Planner(robot)

    driver = FanucDriver(planner, host='192.168.125.100')

    # Create simulated I/O
    driver.create_io_alias('gripper', default_value=0)

    # Read the value of the digital input
    value = driver.get_digital_input('gripper')
    if value is None:
        print("Could not read from address 'gripper'.")
        exit()

    # Set an output for a group of 8 bits
    success = driver.set_digital_output('gripper', 1)
    if not success:
        print("Could not write to address 'gripper'.")
        exit()

    # Read value again
    value = driver.get_digital_input('gripper')
    print(f"Value at digital input 'gripper' is now '{value}'.")
