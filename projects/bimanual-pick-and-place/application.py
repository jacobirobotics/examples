import time
from jacobi import Planner, Studio, MultiRobotPoint, MultiRobotLinearSection, BimanualMotion, LinearSection, Frame, CartesianWaypoint
import numpy as np


class BimanualPickAndPlace:
    def __init__(self, project_file='bimanual-pick-and-place.jacobi-project'):
        # Initialize studio and planner
        self.studio = Studio()
        self.planner = Planner.load_from_project_file(project_file)
        self.yumi = self.planner.environment.get_robots()[0]
        self.yumi.set_speed(0.4)
        self.to_pick, self.to_place = self.get_obstacles_to_pick_and_place()
        self.start = self.calculate_grasp_points(self.yumi, self.to_pick[0])
        self.home = MultiRobotPoint({self.yumi.left: CartesianWaypoint(Frame.from_euler(
            0.5, 0.1, 0.45, np.pi, 0.0, 0.0)), self.yumi.right: CartesianWaypoint(Frame.from_euler(0.5, -0.15, 0.45, np.pi, 0.0, 0.0))})

    def calculate_grasp_points(self, robot, box):
        left_wp = CartesianWaypoint(Frame.from_euler(
            box.origin.translation[0], box.origin.translation[1] + 0.125, box.origin.translation[2], np.pi, 0.0, 0.0))
        right_wp = CartesianWaypoint(Frame.from_euler(
            box.origin.translation[0], box.origin.translation[1] - 0.125, box.origin.translation[2], np.pi, 0.0, 0.0))
        return MultiRobotPoint({robot.left: left_wp, robot.right: right_wp})

    def get_obstacles_to_pick_and_place(self):
        b1 = self.planner.environment.get_obstacle('b1')
        b2 = self.planner.environment.get_obstacle('b2')
        b3 = self.planner.environment.get_obstacle('b3')
        to_pick = [b1, b2, b3]
        a1 = self.planner.environment.get_obstacle('a1')
        a2 = self.planner.environment.get_obstacle('a2')
        a3 = self.planner.environment.get_obstacle('a3')
        to_place = [a1, a2, a3]
        self.planner.environment.remove_obstacle(a1)
        self.planner.environment.remove_obstacle(a2)
        self.planner.environment.remove_obstacle(a3)
        return to_pick, to_place

    def go_to_pick_point(self, box):
        # Disable collision for the box to be picked
        box.for_collision = False
        self.planner.environment.remove_obstacle(box)

        # Calculate a trajectory from the home position to the pick point
        self.start = self.calculate_grasp_points(self.yumi, box)
        motion = BimanualMotion('', self.yumi, self.home, self.start)
        motion.linear_approach = MultiRobotLinearSection({self.yumi.left: LinearSection(offset=Frame.from_translation(
            0, 0, -0.04)), self.yumi.right: LinearSection(offset=Frame.from_translation(0, 0, -0.04))})
        trajectory = self.planner.plan(motion)
        self.studio.run_trajectory(trajectory, robot=self.yumi)
        self.start = MultiRobotPoint(
            {self.yumi.left: trajectory.positions[-1][0:7], self.yumi.right: trajectory.positions[-1][7:14]})

    def pick_and_place_item(self, i):
        b = self.to_pick[i - 1]

        self.go_to_pick_point(b)
        time.sleep(0.1)

        # Calculate a trajectory from the pick point to the home position
        motion = BimanualMotion('', self.yumi, self.start, self.home)
        motion.linear_retraction = MultiRobotLinearSection({self.yumi.left: LinearSection(offset=Frame.from_translation(
            0, 0, -0.05)), self.yumi.right: LinearSection(offset=Frame.from_translation(0, 0, -0.05))})
        motion.is_coordinated = True
        trajectory = self.planner.plan(motion)

        # Calculate a trajectory from the home position to the place point
        goal = self.calculate_grasp_points(self.yumi, self.to_place[i - 1])
        self.planner.environment.remove_obstacle(self.to_place[i - 1])
        self.home = MultiRobotPoint(
            {self.yumi.left: trajectory.positions[-1][0:7], self.yumi.right: trajectory.positions[-1][7:14]})
        motion = BimanualMotion('', self.yumi, self.home, goal)
        motion.is_coordinated = True
        motion.linear_approach = MultiRobotLinearSection({self.yumi.left: LinearSection(offset=Frame.from_translation(
            0, 0, -0.03)), self.yumi.right: LinearSection(offset=Frame.from_translation(0, 0, -0.03))})
        trajectory += self.planner.plan(motion)

        # Before running the trajectory, remove the picked box from the environment
        # and add it as an item to the robot end-effector
        b.for_collision = False
        b.for_visual = False
        self.studio.remove_obstacle(b)
        item = b
        item.name = 'item'
        item.for_visual = True
        item.origin = Frame.from_translation(0.0, 0.125, 0.0)
        self.studio.set_item(item, self.yumi.left)

        # Run the trajectory from pick point to place point
        self.studio.run_trajectory(trajectory, robot=self.yumi)

        # Remove the item from the robot end-effector and place it as an obstacle to the environment
        item.for_visual = False
        self.studio.update_obstacle(item)
        self.to_place[i - 1].for_visual = True
        self.studio.update_obstacle(self.to_place[i - 1])

        # Calculate a trajectory from the place point to the home position
        next_start = MultiRobotPoint({self.yumi.left: trajectory.positions[-1][0:7],
                                      self.yumi.right: trajectory.positions[-1][7:14]})
        motion = BimanualMotion('', self.yumi, next_start, self.home)
        motion.linear_retraction = MultiRobotLinearSection({self.yumi.left: LinearSection(offset=Frame.from_translation(
            0, 0, -0.04)), self.yumi.right: LinearSection(offset=Frame.from_translation(0, 0, -0.04))})
        traj = self.planner.plan(motion)

        # Run the trajectory from place point to the home position
        self.studio.run_trajectory(traj, robot=self.yumi)

    def run_pick_and_place(self):
        for i in range(1, 4):
            self.pick_and_place_item(i)


if __name__ == '__main__':
    bimanual_task = BimanualPickAndPlace()
    bimanual_task.run_pick_and_place()
