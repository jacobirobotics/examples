import math
import copy
import time

from jacobi import Obstacle, Motion, Frame, Planner, Studio, LinearSection, JacobiError


class LabRobot:
    def __init__(self, studio: Studio, planner: Planner, dual_arm):
        self.studio = studio
        self.planner = planner
        self.dual_arm = dual_arm
        self.all_obstacles = planner.environment.get_obstacles()
        for obstacle in self.all_obstacles:
            if 'lab_short_table' in obstacle.name:
                self.table_obs = obstacle
        self._initialize_test_tubes()
        self._initialize_obstacles()
        self._initialize_reference_frames()
        self._initialize_offsets()

    def _initialize_test_tubes(self):
        self.uncapped_test_tubes = [f'test_tube_0_{i}' for i in range(5)]
        self.test_tube_with_caps = [
            f'test_tube_cap_{i}_{j}' for i in range(5) for j in range(5)]

    def _initialize_obstacles(self):
        self.filled_rack = self.planner.environment.get_obstacle('filled_rack')
        self.empty_rack = self.planner.environment.get_obstacle('empty_rack')
        self.cap_obstacle = self.planner.environment.get_obstacle('cap.glb')
        self.test_tube_only_obstacle = self.planner.environment.get_obstacle(
            'test_tube_only_arm')

    def _initialize_reference_frames(self):
        self.left_arm_home_position = [0, -0.52, 0, -1.57, 0, 0, 0]
        self.right_arm_home_position = [
            0.109, -1.489, 0.202, 0.067, -0.886, 2.141, -1.693]

        self.left_arm_idle_frame = Frame(
            x=0.3, y=0.385, z=0.50, a=math.pi / 2, b=math.pi / 4, c=0)
        self.left_arm_center_frame = Frame(
            x=0.42, y=-0.02, z=0.50, a=math.pi / 2, b=math.pi / 4, c=0)

        self.right_arm_idle_frame = Frame(
            x=0.3, y=-0.385, z=0.625, a=-math.pi, b=0, c=-math.pi / 2)
        self.right_arm_center_frame = Frame(
            x=0.42, y=-0.02, z=0.535, a=-math.pi, b=0, c=-math.pi / 2)

    def _initialize_offsets(self):
        self.test_tube_grab_z_offset = 0.035
        self.test_tube_grab_z_offset_rack = 0.132
        self.rack_hole_diameter = 0.0171
        self.rack_hole_clearance = 0.0100
        self.space_between_holes = self.rack_hole_diameter + self.rack_hole_clearance

        self.empty_rack_origin = self.empty_rack.origin * \
            Frame(x=-self.space_between_holes * 2,
                  y=self.space_between_holes * 2)
        self.filled_rack_origin = self.filled_rack.origin * \
            Frame(x=-self.space_between_holes * 2,
                  y=self.space_between_holes * 2)

    def run_trajectory(self, trajectory, robot):
        self.studio.run_trajectory(trajectory, robot=robot)

    def set_to_home(self):
        self.studio.set_joint_position(
            joint_position=self.left_arm_home_position, robot=self.dual_arm.left)
        self.studio.set_joint_position(
            joint_position=self.right_arm_home_position, robot=self.dual_arm.right)

    def arms_go_home(self):
        robot = self.dual_arm
        start = self.studio.get_joint_position(
            robot=self.dual_arm.left) + self.studio.get_joint_position(robot=self.dual_arm.right)
        goal = self.left_arm_home_position + self.right_arm_home_position
        motion_to_go_home = Motion(
            name='goinghome', robot=robot, start=start, goal=goal)
        trajectory_to_go_home = self.planner.plan(motion_to_go_home)
        self.run_trajectory(trajectory_to_go_home, robot=robot)

    def arm_go_home(self, side: str):
        robot = self.dual_arm.left if side == 'left' else self.dual_arm.right
        goal = self.left_arm_home_position if side == 'left' else self.right_arm_home_position

        start = self.studio.get_joint_position(robot=robot)
        motion_to_go_home = Motion(
            name='gohome', robot=robot, start=start, goal=goal)
        trajectory_to_go_home = self.planner.plan(motion_to_go_home)
        self.run_trajectory(trajectory_to_go_home, robot=robot)

    def arms_go_idle(self, side: str):
        robot = self.dual_arm.left if side == 'left' else self.dual_arm.right
        goal = self.left_arm_idle_frame if side == 'left' else self.right_arm_idle_frame

        start = self.studio.get_joint_position(robot=robot)
        motion_to_go_idle = Motion(
            name='goidle', robot=robot, start=start, goal=goal)
        self.planner.environment.add_obstacle(self.table_obs)
        trajectory_to_go_idle = self.planner.plan(motion_to_go_idle)
        self.planner.environment.remove_obstacle(self.table_obs)
        self.run_trajectory(trajectory_to_go_idle, robot=robot)

    def grab_filled_test_tube(self, cap_test_tube: Obstacle):
        robot = self.dual_arm.left
        test_tube_position = cap_test_tube.origin

        start = self.studio.get_joint_position(robot=robot)
        goal = test_tube_position * \
            Frame(z=-self.test_tube_grab_z_offset) * \
            Frame(a=math.pi / 2, b=math.pi / 4, c=0)
        motion_to_grab = Motion(
            name='grab', robot=robot, start=start, goal=goal)
        motion_to_grab.linear_approach = LinearSection(
            offset=Frame(z=-0.05), speed=0.15)
        motion_to_grab.linear_retraction = LinearSection(
            offset=Frame(z=0.001), speed=0.15)

        self._remove_obstacles_for_planning()
        self.planner.environment.remove_obstacle(cap_test_tube)
        trajectory_to_grab = self.planner.plan(motion_to_grab)
        self.run_trajectory(trajectory_to_grab, robot=robot)

        self._attach_test_tube_to_arm(cap_test_tube, robot)
        self._move_arm_to_center(robot)

    def _remove_obstacles_for_planning(self):
        try:
            self.planner.environment.remove_obstacle(
                self.planner.environment.get_obstacle('lab_short_table'))
            self.planner.environment.remove_obstacle(
                self.planner.environment.get_obstacle('filled_rack'))
        # Do not throw error if obstacle is not found
        except JacobiError:
            pass

    def _attach_test_tube_to_arm(self, test_tube: Obstacle, robot):
        self.studio.remove_obstacle(test_tube)
        test_tube.name = 'item'
        test_tube.origin = Frame(x=0.0, y=0.035, z=0.0, a=-1.57, b=0.0, c=0.0)
        robot.item = test_tube
        self.studio.set_item(test_tube, robot=robot)

    def _move_arm_to_center(self, robot):
        start = self.studio.get_joint_position(robot=robot)
        goal = self.left_arm_center_frame
        motion_to_center = Motion(
            name='leftarm2center', robot=robot, start=start, goal=goal)
        motion_to_center.linear_retraction = LinearSection(
            offset=Frame(y=0.2), speed=0.15)
        trajectory = self.planner.plan(motion_to_center)
        self.run_trajectory(trajectory, robot=robot)

    def uncap_test_tube(self):
        robot = self.dual_arm.right
        start = self.studio.get_joint_position(self.dual_arm.right)
        goal = self.right_arm_center_frame
        goal_ik = self.dual_arm.right.inverse_kinematics(goal)
        goal_ik[-1] = goal_ik[-1] + math.pi

        planner_cf = Planner(self.dual_arm.right)
        motion_to_center = Motion(
            name='rightarm2center', robot=robot, start=start, goal=goal_ik)
        motion_to_center.linear_approach = LinearSection(offset=Frame(z=-0.05))
        trajectory_to_center = planner_cf.plan(motion_to_center)
        self.run_trajectory(trajectory_to_center, robot=robot)

        start = self.studio.get_joint_position(robot=robot)
        goal = copy.deepcopy(start)
        goal[-1] = goal[-1] - math.pi
        planner_cf = Planner(self.dual_arm.right)
        motion_to_uncap = Motion(
            name='uncap', robot=robot, start=start, goal=goal)
        trajectory_to_uncap = planner_cf.plan(motion_to_uncap)
        self.run_trajectory(trajectory_to_uncap, robot=robot)

    def switch_obstacles(self):
        self._remove_item_from_arm(self.dual_arm.left)
        self._attach_test_tube_to_arm(
            self.test_tube_only_obstacle, self.dual_arm.left)
        self._attach_cap_to_arm(self.dual_arm.right)

    def _remove_item_from_arm(self, robot):
        robot.item = None
        self.studio.set_item(None, robot=robot)

    def _attach_cap_to_arm(self, robot):
        cap_obs = self.cap_obstacle
        cap_obs.name = 'cap_item'
        cap_obs.origin = Frame(x=0.0, y=0.0, z=0.015, a=3.14, b=0.0, c=0.0)
        robot.item = cap_obs
        self.studio.set_item(cap_obs, robot=robot)

    def right_arm_dispose_cap(self):
        time.sleep(1)
        robot = self.dual_arm.right
        start = self.studio.get_joint_position(robot=robot)
        goal = self.right_arm_idle_frame
        motion_to_dispose = Motion(
            name='dispose', robot=robot, start=start, goal=goal)
        motion_to_dispose.linear_retraction = LinearSection(
            offset=Frame(z=-0.1), speed=0.15)
        planner_cf = Planner(self.dual_arm.right)
        trajectory_to_dispose = planner_cf.plan(motion_to_dispose)
        self.run_trajectory(trajectory_to_dispose, robot=robot)
        self._remove_item_from_arm(self.dual_arm.right)

    def place_test_tube_in_rack(self, row: int, col: int):
        common_goal = self.empty_rack_origin * Frame(x=self.space_between_holes * col, y=self.space_between_holes * row) \
            * Frame(z=self.test_tube_grab_z_offset_rack)
        goal_for_robot = common_goal * \
            Frame(a=-math.pi / 2, b=math.pi, c=math.pi) * Frame(y=-0.015)
        goal_for_tube = common_goal

        robot = self.dual_arm.left
        planner_cf = Planner(robot)
        start = self.studio.get_joint_position(robot=robot)

        motion_to_place = Motion(
            name='place', robot=robot, start=start, goal=goal_for_robot)
        motion_to_place.linear_approach = LinearSection(
            offset=Frame(y=0.1), speed=0.15)
        trajectory_to_place = planner_cf.plan(motion_to_place)
        self.run_trajectory(trajectory_to_place, robot=robot)
        self._remove_item_from_arm(self.dual_arm.left)
        self._spawn_test_tube_in_rack(goal_for_tube)

    def _spawn_test_tube_in_rack(self, goal_for_tube):
        test_tube = self.uncapped_test_tubes.pop(0)
        test_tube = self.planner.environment.get_obstacle(test_tube)
        test_tube.origin = goal_for_tube
        self.studio.add_obstacle(test_tube)


def main():
    project_path = 'lab_automation.jacobi-project'
    time.sleep(4)
    planner = Planner.load_from_project_file(project_path)
    abb_yumi = planner.environment.get_robot('abb_yumi')
    abb_yumi.set_speed(0.6)
    abb_yumi.right.set_speed(0.6)
    abb_yumi.left.set_speed(0.6)
    studio = Studio()
    studio.reset()

    my_lab_robot = LabRobot(studio=studio, planner=planner, dual_arm=abb_yumi)
    my_lab_robot.set_to_home()

    for i in range(4):
        capped_test_tube = my_lab_robot.planner.environment.get_obstacle(
            f'test_tube_cap_0_{i}')
        my_lab_robot.grab_filled_test_tube(capped_test_tube)

        if i == 0:
            my_lab_robot.arms_go_idle('right')

        my_lab_robot.uncap_test_tube()

        my_lab_robot.switch_obstacles()

        my_lab_robot.right_arm_dispose_cap()

        row, col = divmod(i, 5)
        my_lab_robot.place_test_tube_in_rack(row, col)

        my_lab_robot.arms_go_idle('left')

    my_lab_robot.arm_go_home('left')
    print('Lab automation sequence completed successfully!')


if __name__ == '__main__':
    main()
