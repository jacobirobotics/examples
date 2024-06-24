from jacobi import Planner
from jacobi.drivers import SimulatedDriver
from jacobi.robots import UniversalUR5e


if __name__ == '__main__':
    robot = UniversalUR5e()
    planner = Planner(robot)

    driver = SimulatedDriver(planner, serve_io_viewer=True, keep_io_viewer_running=True)
    # driver = SimulatedDriver.Controller(keep_io_viewer_running=True)  # Or as standalone controller with I/Os only

    # Create simulated I/O
    driver.create_io('sensor', default_value=1)
    driver.create_io('gripper', default_value=0)

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
