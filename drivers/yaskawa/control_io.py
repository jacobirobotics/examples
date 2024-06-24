from jacobi import Planner
from jacobi.drivers import YaskawaDriver
from jacobi.robots import YaskawaHC10


if __name__ == '__main__':
    robot = YaskawaHC10()
    planner = Planner(robot, 0.025)

    driver = YaskawaDriver(planner)

    driver.create_io_alias('sensor', address=10)
    driver.create_io_alias('gripper', address=1003)

    # Read the value of the digital input
    value = driver.get_digital_input('sensor')
    if value is None:
        print("Could not read from address 'sensor'.")
        exit()

    print(f"Value at digital input 'sensor' is '{value}'.")

    # Set an output for a group of 8 bits
    success = driver.set_group_output('gripper', 1)
    if not success:
        print("Could not write to address 'gripper'.")
        exit()

    # Read value again
    value = driver.get_digital_input('sensor')
    print(f"Value at digital input 'sensor' is now '{value}'.")
