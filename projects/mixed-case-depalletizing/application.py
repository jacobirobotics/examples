from pathlib import Path

import yaml

from jacobi import Planner, Studio, Obstacle, Box, Frame, Motion, LinearSection, Trajectory


class MixedCaseDepalletizer:
    def __init__(self, project: str):
        self.planner = Planner.load_from_studio(project)
        self.robot = self.planner.environment.get_robot()

        self.box1 = Obstacle('box-1', Box(0.4, 0.2, 0.2), color='E1B471')
        self.box2 = Obstacle('box-2', Box(0.5, 0.4, 0.3), color='E47181')

        self.home = self.planner.environment.get_waypoint('Home')
        self.pallet_left = self.planner.environment.get_obstacle('Pallet Left').origin
        self.pallet_right = self.planner.environment.get_obstacle('Pallet Right').origin
        self.pallet_center = self.planner.environment.get_obstacle('Pallet Center').origin

        self.box_1_pattern = self.load_pattern_from_file('pattern_box1.yml', self.box1.collision)
        self.box_2_pattern = self.load_pattern_from_file('pattern_box2.yml', self.box2.collision)

        self.studio = Studio()
        self.studio.reset()

        self.current_joint_position = self.home.position
        self.studio.set_joint_position(self.current_joint_position)

    def plan_cached(self, m: Motion) -> Trajectory:
        cache_directory = Path('cache/depal')
        cache_directory.mkdir(exist_ok=True, parents=True)

        trajectory_path = cache_directory / f'{m.name}.json'
        if trajectory_path.exists():
            return Trajectory.from_json_file(trajectory_path)

        trajectory = self.planner.plan(m)
        trajectory.to_json_file(trajectory_path)
        return trajectory

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

    def execute_pick_cycle(self, box_at_pick, place_pose):
        # From place to pick
        pick = box_at_pick.origin * Frame(z=box_at_pick.collision.z / 2)

        m = Motion(f'{box_at_pick.name}-to-pick', self.current_joint_position, pick)
        if self.current_joint_position != self.home.position:
            m.linear_retraction = LinearSection(offset=Frame(z=0.05))
        m.linear_approach = LinearSection(offset=Frame(z=0.05))

        trajectory = self.plan_cached(m)
        self.studio.run_trajectory(trajectory)
        self.current_joint_position = trajectory.positions[-1]

        box_as_item = box_at_pick.with_origin(Frame(z=-box_at_pick.collision.z / 2))
        self.studio.set_item(box_as_item)
        self.robot.item_obstacle = box_as_item

        self.studio.remove_obstacle(box_at_pick)
        self.planner.environment.remove_obstacle(box_at_pick)

        # From pick to place
        place_box = self.pallet_center * place_pose
        place = place_box * Frame(z=box_at_pick.collision.z / 2)

        m = Motion(f'{box_at_pick.name}-to-place', self.current_joint_position, place)
        m.orientation_loss_weight = 2.0
        m.linear_retraction = LinearSection(offset=Frame(z=box_at_pick.collision.z + 0.01))
        m.linear_approach = LinearSection(offset=Frame(z=0.05))

        trajectory = self.plan_cached(m)
        self.studio.run_trajectory(trajectory)
        self.current_joint_position = trajectory.positions[-1]

        box_at_place = box_at_pick.with_origin(place_box)
        self.studio.add_obstacle(box_at_place)
        self.planner.environment.add_obstacle(box_at_place)

        self.studio.set_item(None)
        self.robot.item_obstacle = None

    def run(self):
        box_1_obstacles = self.load_pattern_onto_pallet(self.box_1_pattern, self.pallet_left, self.box1)
        box_2_obstacles = self.load_pattern_onto_pallet(self.box_2_pattern, self.pallet_right, self.box2)

        # Define iterators to get the latest picked box
        box_1_stack, box_2_stack = reversed(box_1_obstacles), reversed(box_2_obstacles)

        # Pick and place box 1 and box 2 in arbitrary order and given arbitrary poses on the new pallet
        for place_pose, box_at_pick in zip(self.box_2_pattern[0], box_2_stack):
            self.execute_pick_cycle(box_at_pick, place_pose)

        for place_pose, box_at_pick in zip(reversed(self.box_1_pattern[0][:10]), box_1_stack):
            self.execute_pick_cycle(box_at_pick, place_pose * Frame(z=self.box2.collision.z + 0.01))

        for place_pose, box_at_pick in zip(self.box_2_pattern[1][-2:], box_2_stack):
            self.execute_pick_cycle(box_at_pick, place_pose)

        for place_pose, box_at_pick in zip(reversed(self.box_1_pattern[1][:2]), box_1_stack):
            self.execute_pick_cycle(box_at_pick, place_pose * Frame(y=-0.61, z=self.box2.collision.z + 0.01))

        for place_pose, box_at_pick in zip(reversed(self.box_1_pattern[1][:7]), box_1_stack):
            self.execute_pick_cycle(box_at_pick, place_pose * Frame(z=self.box2.collision.z + 0.01))

        for place_pose, box_at_pick in zip(reversed(self.box_1_pattern[0][-5:]), box_1_stack):
            self.execute_pick_cycle(box_at_pick, place_pose * Frame(z=2 * (self.box2.collision.z + 0.01)))

        # Move back to home
        m = Motion('to-home', self.current_joint_position, self.home)
        trajectory = self.plan_cached(m)
        self.studio.run_trajectory(trajectory)


if __name__ == '__main__':
    application = MixedCaseDepalletizer(project='Mixed-case Depal-pal')
    application.run()
