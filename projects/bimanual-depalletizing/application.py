import time

from pathlib import Path
import yaml
from jacobi import Trajectory, DynamicRobotTrajectory, Frame, Studio, Box, Obstacle, Planner, Motion, LinearSection
from jacobi import BimanualMotion, MultiRobotLinearSection, MultiRobotPoint


class BimanualDepalletizer:
    """
    This class supports three distinct planning modes, each dictating how the robot arms coordinate during pick-and-place operations:

    1. Bimanual Mode ( self.mode = 'bimanual' ):
    - Plans for both arms simultaneously to synchronously perform pick and place actions.
    - Executes the planned trajectories for both arms simultaneously.

    2. Single-Arm Dynamic Mode ( self.mode = 'dynamic' ):
    - Plans the motion for a single arm, then plans the motion for the other arm while considering the first arm's trajectory.
    - Executes the planned trajectories for both arms simultaneously.

    3. Single-Arm Interlock Mode ( self.mode = 'interlock' ):
    - Plans the motion for a single arm, then plans the motion for the other arm considering the first arm's static position.
    - Executes the planned trajectories for both arms sequentially.

    """

    def __init__(self, mode='bimanual'):
        self.mode = mode
        self.planner = Planner.load_from_project_file('bimanual-depalletizing.jacobi-project')
        self.studio = Studio()

        self.pallet_left = self.planner.environment.get_obstacle('pallet_left').origin
        self.pallet_right = self.planner.environment.get_obstacle('pallet_right').origin
        self.box_left = Obstacle('box-left', Box(0.4, 0.2, 0.2), color='E1B471')
        self.box_right = Obstacle('box-right', Box(0.4, 0.2, 0.2), color='E1B471')
        self.box_pattern = self.load_pattern_from_file('pattern_box.yml', self.box_left.collision)

        self.dualarm = self.planner.environment.get_robot('dualarm')

        self.home = self.planner.environment.get_waypoint('home').position
        self.place_pose = self.planner.environment.get_waypoint('place_pose').position

        self.right_current_joint_position = self.home
        self.left_current_joint_position = self.home
        self.studio.set_joint_position(self.home, robot=self.dualarm.left)
        self.studio.set_joint_position(self.home, robot=self.dualarm.right)
        time.sleep(1)

    @staticmethod
    def load_pattern_from_file(path: Path, box: Box) -> list[list[Frame]]:
        """Load a box pattern from a yaml file."""

        with (Path(__file__).absolute().parent / path).open('r') as f:
            data = yaml.safe_load(f)

        def parse(p: dict, axis: str) -> float:
            element = p.get(axis, 0.0)
            if isinstance(element, str):
                return eval(element, {'x': box.x, 'y': box.y, 'z': box.z, 'g': 0.01})
            return element

        def to_frame(p: dict) -> Frame:
            return Frame(x=parse(p, 'x'), y=parse(p, 'y'), z=parse(p, 'z'), c=parse(p, 'c'))

        return [[to_frame(box) for box in layer['boxes']] for layer in data['layers']]

    def plan_cached(self, m: Motion) -> Trajectory:
        cache_directory = Path('cache/depal')
        cache_directory.mkdir(exist_ok=True, parents=True)

        trajectory_path = cache_directory / f'{m.name}.json'
        if trajectory_path.exists():
            return Trajectory.from_json_file(trajectory_path)

        trajectory = self.planner.plan(m)
        trajectory.to_json_file(trajectory_path)
        return trajectory

    def set_item(self, box_to_set, robot):
        box_as_item = box_to_set.with_origin(Frame(z=-box_to_set.collision.z / 2))
        self.studio.set_item(box_as_item, robot)
        robot.item = box_as_item
        self.studio.remove_obstacle(box_to_set)
        self.planner.environment.remove_obstacle(box_to_set)

    def remove_item(self, robot):
        self.studio.set_item(None, robot)
        robot.item = None

    def single_arm_interlock_pick_cycle(self, box_left, box_right):
        # Left arm from place to pick, right arm from pick to place
        pick = box_left.origin * Frame(z=box_left.collision.z / 2)
        place = self.place_pose * box_right.origin.rotation

        m1 = Motion(f'{box_left.name}-to-pick-interlock', self.dualarm.left, self.left_current_joint_position, pick)
        m1.linear_retraction = LinearSection(offset=Frame(z=0.05))
        m1.linear_approach = LinearSection(offset=Frame(z=0.05))
        trajectoryl = self.plan_cached(m1)
        self.studio.run_trajectory(trajectoryl, robot=self.dualarm.left)
        self.left_current_joint_position = trajectoryl.positions[-1]
        self.set_item(box_left, self.dualarm.left)

        m2 = Motion(f'{box_right.name}-to-place-interlock', self.dualarm.right, self.right_current_joint_position, place)
        m2.linear_retraction = LinearSection(offset=Frame(z=box_right.collision.z + 0.01))
        m2.linear_approach = LinearSection(offset=Frame(z=0.05))
        trajectoryr = self.plan_cached(m2)
        self.studio.run_trajectory(trajectoryr, robot=self.dualarm.right)
        self.right_current_joint_position = trajectoryr.positions[-1]
        self.remove_item(self.dualarm.right)

        # Left arm from pick to place, right arm from place to pick
        pick = box_right.origin * Frame(z=box_right.collision.z / 2)
        place = self.place_pose * box_left.origin.rotation

        m1 = Motion(f'{box_right.name}-to-pick-interlock', self.dualarm.right, self.right_current_joint_position, pick)
        m1.linear_retraction = LinearSection(offset=Frame(z=0.05))
        m1.linear_approach = LinearSection(offset=Frame(z=0.05))
        trajectoryr = self.plan_cached(m1)
        self.studio.run_trajectory(trajectoryr, robot=self.dualarm.right)
        self.right_current_joint_position = trajectoryr.positions[-1]
        self.set_item(box_right, self.dualarm.right)

        m2 = Motion(f'{box_left.name}-to-place-interlock', self.dualarm.left, self.left_current_joint_position, place)
        m2.linear_retraction = LinearSection(offset=Frame(z=box_left.collision.z + 0.01))
        m2.linear_approach = LinearSection(offset=Frame(z=0.05))
        trajectoryl = self.plan_cached(m2)
        self.studio.run_trajectory(trajectoryl, robot=self.dualarm.left)
        self.left_current_joint_position = trajectoryl.positions[-1]
        self.remove_item(self.dualarm.left)

    def single_arm_dynamic_pick_cycle(self, box_left, box_right):
        # Left arm from place to pick, right from pick to place
        pick = box_left.origin * Frame(z=box_left.collision.z / 2)
        place = self.place_pose * box_right.origin.rotation

        m1 = Motion(f'{box_left.name}-to-pick-dynamic', self.dualarm.left, self.left_current_joint_position, pick)
        m1.linear_retraction = LinearSection(offset=Frame(z=0.05))
        m1.linear_approach = LinearSection(offset=Frame(z=0.05))
        trajectoryl = self.plan_cached(m1)

        self.planner.dynamic_robot_trajectories = [DynamicRobotTrajectory(trajectoryl, self.dualarm.left)]
        m2 = Motion(f'{box_right.name}-to-place-dynamic', self.dualarm.right, self.right_current_joint_position, place)
        m2.linear_retraction = LinearSection(offset=Frame(z=box_right.collision.z + 0.01))
        m2.linear_approach = LinearSection(offset=Frame(z=0.05))
        trajectoryr = self.plan_cached(m2)
        self.planner.dynamic_robot_trajectories = []
        self.studio.run_trajectories([(trajectoryl, self.dualarm.left), (trajectoryr, self.dualarm.right)])

        self.left_current_joint_position = trajectoryl.positions[-1]
        self.right_current_joint_position = trajectoryr.positions[-1]
        self.set_item(box_left, self.dualarm.left)
        self.remove_item(self.dualarm.right)

        # Left arm from pick to place, right from place to pick
        pick = box_right.origin * Frame(z=box_right.collision.z / 2)
        place = self.place_pose * box_left.origin.rotation

        m1 = Motion(f'{box_right.name}-to-pick-dynamic', self.dualarm.right, self.right_current_joint_position, pick)
        m1.linear_retraction = LinearSection(offset=Frame(z=0.05))
        m1.linear_approach = LinearSection(offset=Frame(z=0.05))
        trajectoryr = self.plan_cached(m1)

        self.planner.dynamic_robot_trajectories = [DynamicRobotTrajectory(trajectoryr, self.dualarm.right)]
        m2 = Motion(f'{box_left.name}-to-place-dynamic', self.dualarm.left, self.left_current_joint_position, place)
        m2.linear_retraction = LinearSection(offset=Frame(z=box_left.collision.z + 0.01))
        m2.linear_approach = LinearSection(offset=Frame(z=0.05))
        trajectoryl = self.plan_cached(m2)
        self.planner.dynamic_robot_trajectories = []
        self.studio.run_trajectories([(trajectoryl, self.dualarm.left), (trajectoryr, self.dualarm.right)])

        self.left_current_joint_position = trajectoryl.positions[-1]
        self.right_current_joint_position = trajectoryr.positions[-1]
        self.set_item(box_right, self.dualarm.right)
        self.remove_item(self.dualarm.left)

    def bimanual_pick_cycle(self, box_left, box_right):
        # Left from place to pick, right from pick to place
        pick = box_left.origin * Frame(z=box_left.collision.z / 2)
        place = self.place_pose * box_right.origin.rotation

        start = MultiRobotPoint({
            self.dualarm.left: self.left_current_joint_position,
            self.dualarm.right: self.right_current_joint_position,
        })
        goal = MultiRobotPoint({
            self.dualarm.left: pick,
            self.dualarm.right: place,
        })
        m = BimanualMotion(f'left-{box_left.name}-right-place', self.dualarm, start, goal)
        m.linear_retraction = MultiRobotLinearSection({
            self.dualarm.left: LinearSection(offset=Frame(z=0.05)),
            self.dualarm.right: LinearSection(offset=Frame(z=box_right.collision.z + 0.01)),
        })
        m.linear_approach = MultiRobotLinearSection({
            self.dualarm.left: LinearSection(offset=Frame(z=0.05)),
            self.dualarm.right: LinearSection(offset=Frame(z=0.05)),
        })

        trajectory = self.plan_cached(m)
        self.studio.run_trajectory(trajectory, robot=self.dualarm)
        self.left_current_joint_position = trajectory.positions[-1][0:6]
        self.right_current_joint_position = trajectory.positions[-1][6:]
        self.set_item(box_left, self.dualarm.left)
        self.remove_item(self.dualarm.right)

        # Left from pick to place, right from place to pick
        pick = box_right.origin * Frame(z=box_right.collision.z / 2)
        place = self.place_pose * box_left.origin.rotation

        start = MultiRobotPoint({
            self.dualarm.left: self.left_current_joint_position,
            self.dualarm.right: self.right_current_joint_position,
        })
        goal = MultiRobotPoint({
            self.dualarm.left: place,
            self.dualarm.right: pick,
        })
        m = BimanualMotion(f'right-{box_right.name}-left-place', self.dualarm, start, goal)
        m.linear_retraction = MultiRobotLinearSection({
            self.dualarm.left: LinearSection(offset=Frame(z=box_left.collision.z + 0.01)),
            self.dualarm.right: LinearSection(offset=Frame(z=0.05)),
        })
        m.linear_approach = MultiRobotLinearSection({
            self.dualarm.left: LinearSection(offset=Frame(z=0.05)),
            self.dualarm.right: LinearSection(offset=Frame(z=0.05)),
        })

        trajectory = self.plan_cached(m)
        self.studio.run_trajectory(trajectory, robot=self.dualarm)
        self.left_current_joint_position = trajectory.positions[-1][0:6]
        self.right_current_joint_position = trajectory.positions[-1][6:]
        self.set_item(box_right, self.dualarm.right)
        self.remove_item(self.dualarm.left)

    def load_pattern_onto_pallet(self, pattern, pallet: Frame, box: Obstacle):
        obstacles = []

        for i_layer, layer in enumerate(pattern):
            for i_box, pose in enumerate(layer):
                b = box.with_origin(pallet * pose)
                b.name = f'{b.name}-{i_layer + 1}-{i_box + 1}'
                self.studio.add_obstacle(b)
                b_ = self.planner.environment.add_obstacle(b)
                obstacles.append(b_)

        return obstacles

    def pick_cycle(self, box_left, box_right):
        if self.mode == 'bimanual':
            self.bimanual_pick_cycle(box_left, box_right)
        elif self.mode == 'dynamic':
            self.single_arm_dynamic_pick_cycle(box_left, box_right)
        elif self.mode == 'interlock':
            self.single_arm_interlock_pick_cycle(box_left, box_right)

    def run(self):
        # Load the box pattern onto the pallets
        box_obstacles_left = self.load_pattern_onto_pallet(self.box_pattern, self.pallet_left, self.box_left)
        box_obstacles_right = self.load_pattern_onto_pallet(self.box_pattern, self.pallet_right, self.box_right)

        # Pick all boxes from the pallets
        for box_l, box_r in zip(reversed(box_obstacles_left), reversed(box_obstacles_right)):
            self.pick_cycle(box_l, box_r)

        # Finish the last box and return to home for both arms
        m_left = Motion('left-to-home', self.dualarm.left, self.left_current_joint_position, self.home)
        t = self.plan_cached(m_left)
        self.studio.run_trajectory(t, robot=self.dualarm.left)

        m_right = Motion('right-place-last-box', self.dualarm.right, self.right_current_joint_position, self.place_pose)
        t = self.plan_cached(m_right)
        self.studio.run_trajectory(t, robot=self.dualarm.right)
        self.right_current_joint_position = t.positions[-1]
        self.remove_item(self.dualarm.right)

        m_right = Motion('right-to-home', self.dualarm.right, self.right_current_joint_position, self.home)
        t = self.plan_cached(m_right)
        self.studio.run_trajectory(t, robot=self.dualarm.right)


if __name__ == '__main__':
    application = BimanualDepalletizer(mode='bimanual')
    application.run()
