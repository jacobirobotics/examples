import math
from pathlib import Path
import numpy as np
import cv2
from PIL import Image

from jacobi import Obstacle, Motion, Frame, Planner, Studio, PointCloud, CartesianWaypoint
from jacobi_vision.drivers import SimulatedCameraDriver
from jacobi_vision import ImageType, ColorImage


class VisionBinPicking:
    def __init__(self, project_path: Path):
        # Load the planner and get references to the robot and camera
        self.planner = Planner.load_from_project_file(project_path)
        self.robot = self.planner.environment.get_robot()
        self.camera = self.planner.environment.get_camera()
        self.camera_driver = SimulatedCameraDriver(self.camera, image_type=ImageType.RGBD)
        self.studio = Studio()
        self.studio.reset()

        # Prepare obstacles for simulated picking
        self.object_for_arm = self.planner.environment.get_obstacle('deodorant_arm')
        self.object_for_carving = self.planner.environment.get_obstacle('deodorant_scaled')
        self.planner.environment.remove_obstacle(self.object_for_carving)
        self.point_cloud_obs = None

        # Remove all "deodorant" obstacles from the environment - we will rely on vision for collision avoidance
        self.obstacles = self.planner.environment.get_obstacles()
        self.obstacles = [o for o in self.obstacles if 'deodorant' in o.name]
        for o in self.obstacles:
            self.planner.environment.remove_obstacle(o)

        # Get references to the boxes we need to pack objects into
        self.boxes_to_pack = [self.planner.environment.get_obstacle(f'box_to_pack_{i}') for i in range(4)]

        # Initialize dimensions and configurations
        self._initialize_dims()

        # Move robot to home configuration at the start
        self.go_home()

    def _initialize_dims(self):
        # Reference joint configurations for picking and placing
        self.reference_config_pick = [0.0, -1.571, 1.571, -1.571, -1.571, 0.0]
        self.reference_config_place = [-1.571, -1.571, 1.571, -1.571, -1.571, 0.0]

        # Adjust rotations for placement orientation
        self.place_rot_adjustment = Frame(a=math.pi, b=0, c=math.pi / 2)

        # Dimensions for the target box and objects
        self.box_to_pack_y = 0.3119882  # [m]
        self.object_width = 0.066675  # [m]
        self.object_height = 0.031496  # [m]

        # Calculate spacing for placing multiple objects in a row
        self.packing_clearance = (self.box_to_pack_y - (4 * self.object_width)) / 5.0

    def get_place_locations(self, box: Obstacle, slot: int):
        # Compute a target Frame for placing the object in the box at a given slot
        place_origin = box.origin * Frame(y=-self.box_to_pack_y / 2) * Frame(z=self.object_height)
        return place_origin * Frame(y=(slot + 1) * self.packing_clearance + (slot + 0.50) * self.object_width)

    def take_snapshot(self):
        # Capture an RGB-D image from the camera
        image = self.camera_driver.get_image()

        # Convert to a point cloud and filter out distant points
        points = image.to_point_cloud().tolist()
        points = [(p[0], p[1], p[2] - 0.002) for p in points if (p[0] < 0.3 and 0.8 < p[2] < 1.5)]
        points = points[::2]  # Downsample points for efficiency

        # Create a point cloud obstacle in the environment
        point_cloud = PointCloud(points, resolution=0.01)
        self.point_cloud_obs = Obstacle(point_cloud, origin=self.camera.origin)
        self.planner.environment.add_obstacle(self.point_cloud_obs)

        # Update the studio visualization of the point cloud
        self.studio.set_camera_point_cloud(np.array(points).flatten().tolist())
        return image

    def go_home(self):
        # Plan and execute a motion to move the robot arm to a "home" configuration
        start = self.studio.get_joint_position()
        motion_to_go_home = Motion(name='gohome', robot=self.robot, start=start, goal=[math.pi / 2, -math.pi / 2, 0, -math.pi / 2, 0, 0])
        trajectory_to_go_home = self.planner.plan(motion_to_go_home)
        self.studio.run_trajectory(trajectory_to_go_home)

    def pick_up_object(self, pose: Frame):
        # Clear any currently held item
        self.robot.item = None

        # Plan a motion to pick the object from the given pose
        start = self.studio.get_joint_position(self.robot)
        motion_to_pick = Motion(name='pick', robot=self.robot, start=start, goal=pose)
        motion_to_pick.robot.item = None

        # Find the closest object and carve it out from the point cloud to help with collision checking
        o = self._find_closest_object(pose)
        self.object_for_carving.origin = Frame(z=0.002) * o.origin
        self.planner.environment.carve_point_cloud(self.point_cloud_obs, self.object_for_carving)
        self.planner.environment.update_point_cloud(self.point_cloud_obs)
        self.studio.set_camera_point_cloud(np.array(self.point_cloud_obs.collision.points).flatten().tolist())

        # Add the arm's object obstacle to help with collision-free planning
        self.planner.environment.add_obstacle(self.object_for_arm)
        trajectory_to_pick = self.planner.plan(motion_to_pick)
        self.studio.run_trajectory(trajectory_to_pick)

        # Hide the original object visually and attach it to the arm
        o.for_visual = False
        self.studio.update_obstacle(o)
        self._attach_object_to_arm()

    def _find_closest_object(self, pose: Frame):
        # Find the obstacle closest to the given pose
        return min(self.obstacles, key=lambda o: pose.translational_distance(o.origin), default=None)

    def _attach_object_to_arm(self):
        # Attach the picked object to the robot's arm for the simulation
        object_obs = self.object_for_arm
        object_obs.name = 'object_item'
        object_obs.origin = Frame(x=0.0, y=-0.069, z=0.015)
        object_obs.for_visual = True
        self.studio.set_item(object_obs)

    def place_object(self, box_to_pack: Obstacle, slot):
        # Plan and execute a motion to place the picked object into a box slot
        start = self.studio.get_joint_position(self.robot)
        place_loc = self.get_place_locations(box_to_pack, slot)
        goal = place_loc * Frame(z=0.01) * self.place_rot_adjustment
        goal_cart = CartesianWaypoint(position=goal, reference_config=self.reference_config_place)

        motion_to_place = Motion(name='place', robot=self.robot, start=start, goal=goal_cart)
        trajectory_to_place = self.planner.plan(motion_to_place)
        self.studio.run_trajectory(trajectory_to_place)

        # Update the scene to show the placed object
        o = self._find_closest_object(self.robot.calculate_tcp(start))
        o.for_visual = True
        o.origin = goal * Frame(y=-0.069)
        self.studio.update_obstacle(o)

        # Release the object from the arm
        self.robot.item = None
        self.studio.set_item(None)

    def detect_objects(self, image):
        # Convert image to OpenCV format and detect objects by color (blue)
        opencv_image = cv2.cvtColor(np.array(Image.fromarray(image.color)), cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find contours of detected objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rectangles = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                if 0.5 < float(w) / h < 2:
                    center_x = x + w / 2
                    center_y = y + h / 2
                    rectangles.append((int(center_x), int(center_y)))

        # Deproject pixel coordinates to 3D points relative to the camera
        points = []
        for i, (x, y) in enumerate(rectangles):
            # Draw detection markers on the debug image
            if i == 0:
                cv2.circle(opencv_image, (x, y), 5, (0, 0, 255), -1)
            cv2.rectangle(opencv_image, (x - 40, y - 70), (x + 40, y + 50), (0, 255, 0), 2)

            # Get the depth and compute 3D coordinates in the scene
            depth = image.depth[y][x]
            p3d = image.deproject(np.array([[y, x, depth]]))[0]
            p3d = self.camera.origin * Frame.from_translation(p3d[1], p3d[0], p3d[2]) * Frame(x=-0.085, y=0.08) * Frame(c=math.pi)
            points.append(p3d)

        # Save the detection result image
        cv2.imwrite('detected_objects.png', opencv_image)
        color_image = ColorImage.load_from_file('detected_objects.png')
        # Set image to the camera in Studio
        encoded_image = color_image.encode()
        self.studio.set_camera_image_encoded(encoded_image, camera=self.camera)

        return points


if __name__ == '__main__':
    project_name = 'bin-picking-with-vision.jacobi-project'
    vision_packer = VisionBinPicking(project_name)

    img = vision_packer.take_snapshot()
    detected_objects = vision_packer.detect_objects(img)
    for j, p in enumerate(detected_objects):
        vision_packer.pick_up_object(p)
        box_index = int(np.floor(j / 4))
        box_slot = j % 4
        vision_packer.place_object(vision_packer.boxes_to_pack[box_index], box_slot)

    vision_packer.go_home()
