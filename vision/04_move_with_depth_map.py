from time import sleep
import numpy as np

from jacobi import Frame, Planner, Studio, Intrinsics, DepthMap, Camera, Environment
from jacobi.robots import FrankaPanda


if __name__ == '__main__':
    # 1. Create Studio live connection
    studio = Studio()
    studio.reset()

    # 2. Define the robot, environment, and planner
    robot = FrankaPanda()
    robot.base = Frame(x=-0.5)
    studio.add_robot(robot)
    sleep(3.0)

    environment = Environment(robot)
    planner = Planner(environment)

    # 3. Define the camera
    intrinsics = Intrinsics(500.0, 500.0, optical_center_x=320, optical_center_y=240, width=640, height=480)
    camera = Camera('model', 'Camera', Frame(z=2.0, a=3.1415), intrinsics)

    studio.add_camera(camera)

    # 4. Define start and goal waypoints
    start = Frame(x=-0.2, y=0.6, z=0.5, a=3.1415)
    goal = Frame(x=-0.2, y=-0.6, z=0.5, a=3.1415)

    studio.add_waypoint(start)
    studio.add_waypoint(goal)

    # 5. Define the depth map collision object
    # 5.1 A flat depth map obstacle
    depth_map_flat = np.full((20, 20), 2.0)
    depth_map_flat[1:-1, 1:-1] = 1.8

    # 5.2 And a higher one that the robot actively needs to avoid
    depth_map_object = np.full((20, 20), 2.0)
    depth_map_object[1:-1, 5:12] = 1.4
    depth_map_object[1:-1, 12:-1] = 1.1

    map_obstacle = environment.add_obstacle(DepthMap(depth_map_object, 0.8, 0.6), origin=camera.origin)
    studio.set_camera_depth_map(depth_map_object, 0.8, 0.6)

    # 6. Plan a motion from start to goal
    trajectory = planner.plan(start, goal)
    studio.run_trajectory(trajectory)

    # 7. Plan a motion back with a new depth map
    map_obstacle.collision.depths = depth_map_flat
    environment.update_depth_map(map_obstacle)
    studio.set_camera_depth_map(depth_map_flat, 0.8, 0.6)

    trajectory = planner.plan(goal, start)
    studio.run_trajectory(trajectory)
