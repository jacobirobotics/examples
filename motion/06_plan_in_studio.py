from jacobi import Planner


if __name__ == '__main__':
    # 1. Set up planner by passing the project name in Jacobi Studio
    planner = Planner.load_from_studio('My Project')

    # 2. Plan motion defined in Studio project
    trajectory = planner.plan('Home to Camera')

    # 3. Plan with new start and goal positions
    # trajectory = planner.plan(start=[1.6, 0.4, 0.1, 0, 0.5, -1.5], goal=[0.0, 0.4, 0.6, 0.0, 0.4, -0.4])

    # 4. Print returned trajectory
    if not trajectory:
        raise RuntimeError('Could not calculate a valid trajectory.')

    print(f'Trajectory duration: {trajectory.duration:0.3f} [s]')
    print(f'First joint positions: {trajectory.positions[:2]}')
