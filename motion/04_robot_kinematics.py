from jacobi import Frame
from jacobi.robots import YaskawaGP12


if __name__ == '__main__':
    # 1. Set up the robot, and define the base frame as well as the flange to TCP transformation
    robot = YaskawaGP12()
    robot.base = Frame(z=0.765)
    robot.flange_to_tcp = Frame(z=0.2)

    # 2. Forward Kinematics
    home_config = [-1.57, -0.7042, -0.4648, 0.0, -1.809, 0.0]
    home_pose = robot.calculate_tcp(home_config)

    print(f'Home translation: {home_pose}')

    # 3. Inverse Kinematics
    grasp_pose = Frame.from_euler(0.4, 0.1, 0.4, 0.0, 0.0, -1.2)
    nearest_joint_position = [-1.57, -0.7042, -0.4648, 0.0, -1.809, 0.0]

    grasp_joint_position = robot.inverse_kinematics(grasp_pose, nearest_joint_position)
    if grasp_joint_position is None:
        print('Could not find a inverse solution!')
        exit()

    print(f'Joint configuration: {grasp_joint_position}')
