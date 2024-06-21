import asyncio
from pathlib import Path

from jacobi import Frame, Motion, LinearMotion, LinearSection, Planner, Studio


class AssemblyApplication:
    project_path = Path.home() / 'Downloads' / 'assembly.jacobi-project'

    def __init__(self):
        self.planner = Planner.load_from_project_file(self.project_path)
        self.environment = self.planner.environment
        self.robot = self.environment.get_robot()

        self.home = self.environment.get_waypoint('Home')
        self.assembly = self.environment.get_waypoint('Assembly')

        self.cylinders = [
            self.environment.get_obstacle('Cylinder 1'),
            self.environment.get_obstacle('Cylinder 2'),
            self.environment.get_obstacle('Cylinder 3'),
            self.environment.get_obstacle('Cylinder 4'),
        ]

        # Just for visualization
        self.studio = Studio()
        self.robot_position = [0, 0, 0, 0, 0, 0]  # Initial joint position [rad]

        # TODO: Set up robot driver
        # self.driver = FanucDriver(host='192.168.1.135')

    async def run_trajectory(self, trajectory, events=Studio.Events()):
        # TODO: Use actual robot driver
        # self.driver.run(trajectory)

        self.studio.run_trajectory(trajectory, events)  # requires the Studio Live feature
        self.robot_position = trajectory.positions[-1]

    async def grasp(self):
        # TODO: Grasp cylinder
        await asyncio.sleep(0.5)  # [s]
        # TODO: Check input condition

    async def release(self):
        # TODO: Release cylinder
        await asyncio.sleep(0.5)  # [s]
        # TODO: Check input condition

    async def assemble_cylinder(self, cylinder, retract_from_start=True):
        # 1. Move from current position -> Cylinder pick
        pick_frame = cylinder.origin
        motion = Motion(self.robot_position, pick_frame)
        motion.linear_approach = LinearSection(Frame(z=0.1), speed=0.5)  # [m]
        if retract_from_start:
            motion.linear_retraction = LinearSection(Frame(z=0.05), speed=0.5)  # [m]
        trajectory = self.planner.plan(motion)
        await self.run_trajectory(trajectory)

        # 2. Grasp cylinder
        await self.grasp()

        cylinder.origin = Frame.Identity()
        self.robot.item_obstacle = cylinder

        events = Studio.Events()
        events[0.0] = Studio.Events.set_item(self.robot.item_obstacle)
        events[0.0] = Studio.Events.remove_obstacle(cylinder)

        # 3. Move from pick -> Angled assembly approach
        assembly_frame = self.assembly.position
        assembly_frame_approach = self.assembly.position * Frame(a=0.18)  # [rad]

        motion = Motion(self.robot_position, assembly_frame_approach)
        motion.linear_retraction = LinearSection(Frame(z=0.1), speed=0.5)  # [m]
        motion.linear_approach = LinearSection(Frame(z=0.05), speed=0.1)  # [m]
        trajectory = self.planner.plan(motion)
        await self.run_trajectory(trajectory, events)

        # 4. Orient normal to fixture
        motion = LinearMotion(self.robot_position, assembly_frame)
        trajectory = self.planner.plan(motion)

        events = Studio.Events()
        cylinder.origin = self.assembly.position
        events[trajectory.duration] = Studio.Events.add_obstacle(cylinder)
        events[trajectory.duration] = Studio.Events.set_item(None)
        await self.run_trajectory(trajectory, events)

        # 5. Release cylinder
        await self.release()

        self.robot.item_obstacle = None

    async def run(self):
        # 0. Move to home position
        trajectory = self.planner.plan(self.robot_position, self.home)
        await self.run_trajectory(trajectory)

        for i, cylinder in enumerate(self.cylinders):
            await self.assemble_cylinder(cylinder, retract_from_start=i > 0)

        # End. Move to home position again
        motion = Motion(self.robot_position, self.home)
        motion.linear_retraction = LinearSection(Frame(z=0.1), speed=0.5)  # [m]
        trajectory = self.planner.plan(motion)
        await self.run_trajectory(trajectory)

        self.studio.reset()  # Reset the visualization


if __name__ == '__main__':
    application = AssemblyApplication()
    asyncio.run(application.run())
