from jacobi import Environment, Planner
from jacobi.robots import ABBYuMiIRB14000


if __name__ == '__main__':
    # 1. Set up the robot, transformations, and its kinematics limits
    yumi = ABBYuMiIRB14000()
    yumi.set_speed(0.2)

    # 2. Calculate forward and inverse kinematics of left and right arms
    tcp_right = yumi.right.calculate_tcp([0, -0.3, 0, 0.2, 0, 0.1, 0])
    print(f'Right TCP Position: {tcp_right}')

    tcp_left = yumi.left.calculate_tcp([0, -0.4, 0, 0, 0, 0, 0])
    print(f'Left TCP Position: {tcp_left}')

    joint_position_right = yumi.right.inverse_kinematics(tcp_left)
    print(f'Right Joint Position to Reach Left TCP: {joint_position_right}')

    # 3. Check for collision
    environment = Environment(yumi)

    in_collision = environment.check_collision([0, -0.2, 0, 0, 0, 0, 0] + [0, 0.3, 0, 0, 0, 0, 0])
    print(f'In collision? {in_collision}')

    # 4.1 Set up the joint planner that considers left <-> right collisions
    planner = Planner(environment)

    trajectory = planner.plan(
        start=[0, -0.3, 0, 0, 0, 0, 0] + [0, 0.3, 0, 0, 0, 0, 0],
        goal=[0, 0.3, 0, 0.2, 0, 0, 0] + [0, -0.5, 0, 0.4, 0, 0, 0],
    )

    # 4.2 Set up the planner for a single robot arm (without left <-> right collision checking)
    # planner_left = Planner(yumi.left)

    # trajectory = planner_left.plan(
    #     start=[0, -0.3, 0, 0, 0, 0, 0],
    #     goal=[0, 0.3, 0, 0.2, 0, 0, 0],
    # )

    # 5. Print returned trajectory
    if not trajectory:
        print('Could not find a trajectory!')
        exit()

    print(f'Trajectory duration: {trajectory.duration:0.3f} [s]')
    print(f'Calculation duration: {planner.last_calculation_duration:0.2f} [ms]')
