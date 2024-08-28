from pathlib import Path
import time

import numpy as np

from jacobi import Frame, Planner, JacobiError, PathFollowingMotion, LinearPath, CircularPath, BlendedPath, Studio


class InspectionProject:
    def __init__(self, planner, studio):
        self.planner = planner
        self.studio = studio
        self.object_list = [planner.environment.get_obstacle(o) for o in ['laptop', 'console', 'phone']]

    def go_home(self):
        """Move the robot to the home position."""

        home = [1.347, -1.208, 1.759, 4.16, -1.571, -0.224]
        traj = self.planner.plan(self.studio.get_joint_position(), home)
        self.studio.run_trajectory(traj)
        time.sleep(0.1)

    def visualize_path(self, path_type):
        """Visualize the calculated path of the robot tool with the attached camera in Studio."""

        path = path_type.calculate_path(0.3, 0.01)
        p = [path[i].translation for i in range(len(path))]
        self.studio.add_robot_path(p)

    def inspect_laptop(self):
        """Inspect the laptop object using a blended path."""

        # Move from the home position to the inspection start position
        start = Frame(x=-0.16, y=0.44, z=0.89, a=-2.3)
        traj = self.planner.plan(self.studio.get_joint_position(), start)
        self.studio.run_trajectory(traj)

        # Define the waypoints and robot tool velocity for the inspection path
        velocity = 0.3  # [m/s]
        waypoints = [
            start,
            Frame(x=0.17, y=0.44, z=0.89, a=-2.3),
            Frame(x=0.17, y=0.48, z=0.89, a=-2.5),
            Frame(x=-0.16, y=0.48, z=0.89, a=-2.5),
            Frame(x=-0.16, y=0.52, z=0.89, a=-2.7),
            Frame(x=0.17, y=0.52, z=0.89, a=-2.7),
            Frame(x=0.17, y=0.56, z=0.89, a=-2.9),
            Frame(x=-0.16, y=0.56, z=0.89, a=-2.9),
            Frame(x=-0.16, y=0.60, z=0.89, a=-3.10),
            Frame(x=0.17, y=0.60, z=0.89, a=-3.10),
            Frame(x=0.17, y=0.64, z=0.89, a=-3.3),
            Frame(x=-0.16, y=0.64, z=0.89, a=-3.3),
            Frame(x=-0.16, y=0.68, z=0.89, a=-3.5),
            Frame(x=0.17, y=0.68, z=0.89, a=-3.5),
            Frame(x=0.17, y=0.72, z=0.89, a=-3.7),
            Frame(x=-0.16, y=0.72, z=0.89, a=-3.7),
            Frame(x=-0.16, y=0.76, z=0.89, a=-3.9),
            Frame(x=0.17, y=0.76, z=0.89, a=-3.9),
        ]

        # Create the blended path and visualize it in Studio
        path = BlendedPath(waypoints, 0.01)
        self.visualize_path(path)

        # Create the motion object and set the reference configuration
        motion = PathFollowingMotion(path, velocity)
        motion.reference_config = self.studio.get_joint_position()

        # Plan the trajectory and run it in Studio
        traj = self.planner.plan(motion)
        self.studio.run_trajectory(traj)
        self.studio.reset()
        time.sleep(0.4)

    def inspect_console(self):
        """Inspect the console object using a circular path."""

        # Move from the home position to the inspection start position
        start = Frame(y=0.45, z=1.0, a=-2.6)
        traj = self.planner.plan(self.studio.get_joint_position(), start)
        self.studio.run_trajectory(traj)

        # Define the first circular path parameters and robot tool velocity
        velocity = 0.12  # [m/s]
        theta = 2 * np.pi
        center = [0.0, 0.61, 1.0]
        normal = [0.0, 0.0, 1.0]

        # Create the circular path and visualize it in Studio
        path = CircularPath(start, theta, center, normal, True)
        self.visualize_path(path)

        # Create the motion object, set the reference configuration and plan the first motion
        motion = PathFollowingMotion(path, velocity)
        motion.reference_config = self.studio.get_joint_position()
        traj = self.planner.plan(motion)

        # Create the linear path to move the robot tool to the next inspection position and visualize it
        pos = self.planner.environment.get_robot().calculate_tcp(traj.positions[-1])
        path = LinearPath(pos, Frame(y=0.45, z=0.85, a=-2.4))
        self.visualize_path(path)

        # Create the motion object, set the reference configuration and plan the second motion
        move_to_next = PathFollowingMotion(path, velocity)
        move_to_next.reference_config = traj.positions[-1]
        traj += self.planner.plan(move_to_next)

        # Define the second circular path parameters
        start = Frame(y=0.45, z=0.85, a=-2.4)
        center = [0.0, 0.61, 0.85]
        normal = [0.0, 0.0, -1.0]

        # Create the circular path and visualize it in Studio
        path = CircularPath(start, theta, center, normal, True)
        self.visualize_path(path)

        # Create the motion object, set the reference configuration and plan the third motion
        motion = PathFollowingMotion(path, velocity)
        motion.reference_config = traj.positions[-1]
        traj += self.planner.plan(motion)

        # Run the full trajectory in Studio
        self.studio.run_trajectory(traj)
        self.studio.reset()
        time.sleep(0.4)

    def inspect_phone(self):
        """Inspect the phone case object using a blended path."""

        # Move from the home position to the inspection start position
        start = Frame(x=-0.05, y=0.55, z=0.85, a=3.14)
        traj = self.planner.plan(self.studio.get_joint_position(), start)
        self.studio.run_trajectory(traj)

        # Define the waypoints and robot tool velocity for the inspection path
        velocity = 0.05  # [m/s]
        waypoints = [
            start,
            Frame(x=0.05, y=0.55, z=0.85, a=3.14),
            Frame(x=0.05, y=0.6, z=0.85, a=3.14),
            Frame(x=-0.05, y=0.6, z=0.85, a=3.14),
            Frame(x=-0.05, y=0.65, z=0.85, a=3.14),
            Frame(x=0.05, y=0.65, z=0.85, a=3.14),
        ]

        # Create the blended path and visualize it in Studio
        path = BlendedPath(waypoints, 0.01)
        self.visualize_path(path)

        # Create the motion object and set the reference configuration
        motion = PathFollowingMotion(path, velocity)
        motion.reference_config = self.studio.get_joint_position()

        # Plan the trajectory and run it in Studio
        traj = self.planner.plan(motion)
        self.studio.run_trajectory(traj)
        self.studio.reset()
        time.sleep(0.4)

    def switch_obstacle(self, name_visible):
        """Switch the view and collision checking to the specified obstacle."""

        for o in self.object_list:
            if o.name == name_visible:
                o.for_visual = True
                self.planner.environment.add_obstacle(o)
            else:
                try:  # noqa: SIM105
                    self.planner.environment.remove_obstacle(self.planner.environment.get_obstacle(o.name))
                except JacobiError:
                    pass
                o.for_visual = False
            self.studio.update_obstacle(o)
        time.sleep(0.5)

    def run(self):
        """Run the inspection project."""

        self.go_home()

        self.switch_obstacle('laptop')
        self.inspect_laptop()
        self.go_home()

        self.switch_obstacle('console')
        self.inspect_console()
        self.go_home()

        self.switch_obstacle('phone')
        self.inspect_phone()
        self.go_home()


if __name__ == '__main__':
    # Load project from Studio Download
    project_path = Path.home() / 'Downloads' / 'Inspection-UR5e.jacobi-project'

    # Run the inspection project
    inspection = InspectionProject(
        planner=Planner.load_from_project_file(project_path),
        studio=Studio(),
    )
    inspection.run()
