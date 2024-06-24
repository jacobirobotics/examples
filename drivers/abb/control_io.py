from jacobi import Planner
from jacobi.robots import ABBIRB1200590
from jacobi.drivers import ABBDriver


if __name__ == '__main__':
    robot = ABBIRB1200590()
    planner = Planner(robot)

    driver = ABBDriver(planner, host='192.168.125.1', port=6511)

    # Read the value of the digital input
    value = driver.get_digital_input('gripper')
    if value is None:
        print("Could not read from address 'gripper'.")
        exit()

    print(f"Value at digital input 'sensor' is '{value}'.")

    # Set an output for a group of 8 bits
    success = driver.set_digital_output('gripper', 1)
    if not success:
        print("Could not write to address 'gripper'.")
        exit()

    # Read value again
    value = driver.get_digital_input('gripper')
    print(f"Value at digital input 'gripper' is now '{value}'.")
