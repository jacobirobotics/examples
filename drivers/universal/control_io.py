from jacobi import Planner
from jacobi.drivers import UniversalDriver
from jacobi.robots import UniversalUR10e


if __name__ == '__main__':
    robot = UniversalUR10e()
    planner = Planner(robot)

    driver = UniversalDriver(planner, '192.168.102.132')

    driver.create_io_alias('sensor', address=0)
    driver.create_io_alias('gripper', address=0)

    # Read the value of the digital input
    value = driver.get_digital_input('sensor')
    if value is None:
        print("Could not read from address 'sensor'.")
        exit()

    print(f"Value at digital input 'sensor' is '{value}'.")

    # Set an output for a group of 8 bits
    success = driver.set_digital_output('gripper', 1)
    if not success:
        print("Could not write to address 'gripper'.")
        exit()

    # Read value again
    value = driver.get_digital_input('sensor')
    print(f"Value at digital input 'sensor' is now '{value}'.")
