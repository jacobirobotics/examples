from dataclasses import dataclass
import math
import random

from jacobi import Box, Obstacle, Frame, Motion, LinearSection, Planner
from jacobi.drivers import SimulatedDriver


class Container:
    """State of a container to get the next box position."""

    def __init__(self, per_layer: int, number_layers: int) -> None:
        self.per_layer = per_layer
        self.number_layers = number_layers
        self.capacity = self.per_layer * self.number_layers
        self.available_spaces = self.capacity
        self.next_position = {'layer': 0, 'box_in_layer': 0}

    def add(self) -> tuple[int, int]:
        if self.available_spaces <= 0:
            return None

        position = self.next_position.copy()
        self.available_spaces -= 1

        # Update next_position
        self.next_position['box_in_layer'] += 1
        if self.next_position['box_in_layer'] == self.per_layer:
            self.next_position['box_in_layer'] = 0
            self.next_position['layer'] += 1

        return position['layer'], position['box_in_layer']


@dataclass
class BoxesForContainer:
    """All info about palletizing a box into a container."""

    box: Obstacle
    obstacle: Obstacle
    container: Container
    place_frames: list[Frame]
    place_linear: list[Frame]

    @staticmethod
    def generate_box_order(number_small: int, info_small, number_big: int, info_big) -> list:
        box_order = [info_small] * number_small + [info_big] * number_big
        random.shuffle(box_order)
        return box_order


def run_application():
    # Load project from Studio
    planner = Planner.load_from_studio('Cage and Gaylord Palletizing')
    robot = planner.environment.get_robot()

    # Get waypoints for motion planning
    conveyor_feed_rot = math.radians(90)
    home = planner.environment.get_waypoint('Home')
    surface_pick = planner.environment.get_waypoint('Pick')
    roller_cage = planner.environment.get_obstacle('cage_bottom_collision')
    gaylord_box = planner.environment.get_obstacle('GL_bottom_collision')

    # Boxes and containers
    container_roller_cage = Container(per_layer=4, number_layers=4)
    container_gaylord_box = Container(per_layer=3, number_layers=5)

    # Clearances
    stack_clearance = 0.001  # [m]
    rc_clearance = 0.0175  # [m]
    gl_clearance = 0.0175  # [m]

    # Dimensions and box positions
    small_box_x = (roller_cage.collision.x - rc_clearance * (container_roller_cage.per_layer + 1)) / (container_roller_cage.per_layer)
    big_box_x = (gaylord_box.collision.x - gl_clearance * (container_gaylord_box.per_layer + 1)) / (container_gaylord_box.per_layer)
    small_box_geometry = Box(small_box_x, roller_cage.collision.y - 0.150, 0.380)
    big_box_geometry = Box(big_box_x, gaylord_box.collision.y - 0.075, 0.1675)

    roller_cage_frames = [
        Frame(x=((roller_cage.collision.x / 2) - (1 * rc_clearance) - (small_box_geometry.x / 2))),
        Frame(x=((roller_cage.collision.x / 2) - (2 * rc_clearance) - (3 * small_box_geometry.x / 2))),
        Frame(x=((roller_cage.collision.x / 2) - (3 * rc_clearance) - (5 * small_box_geometry.x / 2))),
        Frame(x=((roller_cage.collision.x / 2) - (4 * rc_clearance) - (7 * small_box_geometry.x / 2))),
    ]

    gaylord_box_frames = [
        Frame(x=((gaylord_box.collision.x / 2) - (1 * gl_clearance) - (big_box_geometry.x / 2))),
        Frame(x=((gaylord_box.collision.x / 2) - (2 * gl_clearance) - (3 * big_box_geometry.x / 2))),
        Frame(x=((gaylord_box.collision.x / 2) - (3 * gl_clearance) - (5 * big_box_geometry.x / 2))),
    ]

    # Linear retraction/approach parameters, layer-wise
    pick_linear = Frame(z=0.05)  # [m]
    rc_place_linear = [Frame(z=0.650), Frame(z=0.500), Frame(z=0.350), Frame(z=0.200), Frame(z=0.150), Frame(z=0.125)]  # [m]
    gl_place_linear = [Frame(z=0.490), Frame(z=0.450), Frame(z=0.380), Frame(z=0.345), Frame(z=0.175), Frame(z=0.150)]  # [m]

    # Container info
    info_small = BoxesForContainer(
        box=Obstacle('small_box', small_box_geometry, Frame(), color='B99976'),
        obstacle=roller_cage,
        container=container_roller_cage,
        place_frames=roller_cage_frames,
        place_linear=rc_place_linear,
    )

    info_big = BoxesForContainer(
        box=Obstacle('big_box', big_box_geometry, Frame(), color='987554'),
        obstacle=gaylord_box,
        container=container_gaylord_box,
        place_frames=gaylord_box_frames,
        place_linear=gl_place_linear,
    )

    box_order = BoxesForContainer.generate_box_order(
        number_small=container_roller_cage.capacity, info_small=info_small,
        number_big=container_gaylord_box.capacity, info_big=info_big,
    )

    # Set and reset Studio project for visualization
    driver = SimulatedDriver(planner, sync_with_studio=True)
    driver.set_current_joint_position(home.position)
    driver.studio.reset()
    driver.studio.set_item(None)

    for idx, info in enumerate(box_order):
        # Pick position, top of box surface
        box_height = info.box.collision.z
        info.box.name = f'box_{idx}'

        pick = surface_pick.position * Frame(z=box_height)
        box_at_pick = info.box.with_origin(pick * Frame(z=-box_height / 2, c=conveyor_feed_rot))
        driver.studio.add_obstacle(box_at_pick)  # spawn box at pick

        # Finding place location
        layer, box_in_layer = info.container.add()
        layer_frame = info.obstacle.origin * Frame(z=info.obstacle.collision.z / 2) * Frame(z=(box_height + stack_clearance) * layer)
        place_frame = layer_frame * info.place_frames[box_in_layer] * Frame(z=box_height + stack_clearance)

        if idx == 0:
            # Define home -> pick motion
            motion = Motion('home_to_pick', driver.current_joint_position, pick)
            motion.linear_approach = LinearSection(offset=Frame(z=0.20), speed=0.15)

        else:
            # Define place -> pick motion
            motion = Motion(f'place_to_pick_{idx}', driver.current_joint_position, pick)
            motion.linear_retraction = LinearSection(offset=info.place_linear[layer])
            motion.linear_approach = LinearSection(offset=pick_linear)

        # Plan and execute pick -> place motion
        trajectory = planner.plan(motion)
        driver.run(trajectory)

        # Grasp the box, and update collision model and Studio
        driver.studio.remove_obstacle(box_at_pick)
        box_as_item = info.box.with_origin(Frame(z=-box_height / 2, c=conveyor_feed_rot))
        robot.item_obstacle = box_as_item
        driver.studio.set_item(box_as_item)

        # Define pick -> place motion
        motion = Motion(f'pick_to_place_{idx}', driver.current_joint_position, place_frame * Frame(c=-conveyor_feed_rot))
        motion.linear_retraction = LinearSection(offset=pick_linear)
        motion.linear_approach = LinearSection(offset=info.place_linear[layer])
        motion.orientation_loss_weight = 2.0
        motion.path_length_loss_weight = 2.0

        # Plan and execute pick -> place motion
        trajectory = planner.plan(motion)
        driver.run(trajectory)

        # Release box from the robot
        robot.item_obstacle = None
        driver.studio.set_item(None)

        # Add box at place position to collision model and Studio
        box_at_place = info.box.with_origin(place_frame * Frame(z=-box_height / 2))
        planner.environment.add_obstacle(box_at_place)
        driver.studio.add_obstacle(box_at_place)


if __name__ == '__main__':
    run_application()
