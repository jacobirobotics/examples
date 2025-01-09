from pathlib import Path

from jacobi import Box, Obstacle, Frame, Motion, LinearSection, Planner
from jacobi.drivers import SimulatedDriver


if __name__ == '__main__':
    # Load project from Studio Download
    project_path = Path.home() / 'Downloads' / 'TechCrunch Video.jacobi-project'
    planner = Planner.load_from_project_file(project_path)
    robot = planner.environment.get_robot()

    # Get important waypoints for motion planning
    home = planner.environment.get_waypoint('Home')
    pick = planner.environment.get_waypoint('Pick')
    pallet = planner.environment.get_obstacle('Pallet Left')

    # Define box properties
    box_geometry = Box(0.4, 0.34, 0.3)
    box_color = 'C09B6A'
    pick = pick.position * Frame(z=box_geometry.z)

    # Set and reset Studio project for visualization
    driver = SimulatedDriver(planner, sync_with_studio=True)
    driver.speed = 24.0
    driver.set_current_joint_position(home.position)
    driver.studio.reset()
    driver.studio.set_item(None)

    # Add a first box on the conveyor belt
    box_at_pick = Obstacle('box_at_pick', box_geometry, pick * Frame(z=-box_geometry.z / 2), color=box_color)
    driver.studio.add_obstacle(box_at_pick)

    # Going from home to the first pick
    motion = Motion('home_to_pick', driver.current_joint_position, pick)
    motion.linear_approach = LinearSection(offset=Frame(z=0.05), speed=0.5)
    trajectory = planner.plan(motion)
    driver.run(trajectory)

    for layer_i in range(5):
        layer = pallet.origin * Frame(z=pallet.collision.z / 2) * Frame(z=(box_geometry.z + 0.005) * layer_i)

        for box_i, box_frame in enumerate([
            Frame(x=-0.41, y=-0.18 - 0.04),
            Frame(x=0.0, y=-0.18 - 0.04),
            Frame(x=0.41, y=-0.18 - 0.04),
            Frame(x=-0.41, y=0.18 - 0.04),
            Frame(x=0.0, y=0.18 - 0.04),
            Frame(x=0.41, y=0.18 - 0.04),
        ]):
            # Grasp the box, and update collision model and Studio
            driver.studio.remove_obstacle(box_at_pick)
            box_as_item = Obstacle('box_as_item', box_geometry, Frame(z=-box_geometry.z / 2), color=box_color)
            robot.item = box_as_item
            driver.studio.set_item(box_as_item)

            # Calculate place position on pallet
            place = layer * box_frame * Frame(z=box_geometry.z)

            # Calculate the pick -> place motion
            motion = Motion(f'{layer_i + 1}_{box_i + 1}_pick_to_place', driver.current_joint_position, place)
            motion.linear_retraction = LinearSection(offset=Frame(z=0.2))
            motion.linear_approach = LinearSection(offset=Frame(x=0.1, y=0.1, z=0.1))
            motion.orientation_loss_weight = 2.0
            trajectory = planner.plan(motion)
            driver.run(trajectory)

            # Release box
            robot.item = None
            driver.studio.set_item(None)

            # Add box at place position to collision model and Studio
            box_at_place_frame = place * Frame(z=-box_geometry.z / 2)
            box_at_place = Obstacle(f'Box {box_i + 1} - Layer {layer_i + 1} ', box_geometry, box_at_place_frame, color=box_color)
            planner.environment.add_obstacle(box_at_place)
            driver.studio.add_obstacle(box_at_place)
            driver.studio.add_obstacle(box_at_pick)

            # Calculate the place -> pick return motion
            motion = Motion(f'{layer_i + 1}_{box_i + 1}_place_to_pick', driver.current_joint_position, pick)
            motion.linear_retraction = LinearSection(offset=Frame(z=0.05))
            motion.linear_approach = LinearSection(offset=Frame(z=0.1))
            trajectory = planner.plan(motion)
            driver.run(trajectory)
