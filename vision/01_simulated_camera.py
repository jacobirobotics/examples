# Open an empty Studio project to run this example

import matplotlib.pyplot as plt

from jacobi import Camera, Intrinsics, Studio, Frame, Obstacle, Box
from jacobi_vision.drivers import SimulatedCameraDriver


if __name__ == '__main__':
    studio = Studio()
    studio.reset()

    # 1. Define camera (or add to Studio project via GUI)
    intrinsics = Intrinsics(500.0, 500.0, optical_center_x=320, optical_center_y=240, width=640, height=480)
    camera = Camera('model', 'Camera', Frame(z=2.0, a=3.1415), intrinsics)

    studio.add_camera(camera)

    # 2. Also add a few obstacles to the scene
    studio.add_obstacle(Obstacle(Box(0.4, 0.7, 0.2), Frame(x=0.3), color='ff0000'))
    studio.add_obstacle(Obstacle(Box(0.3, 0.4, 0.5), Frame(x=-0.5, z=0.3)))

    # 3. Create camera driver
    driver = SimulatedCameraDriver(camera, studio=studio)

    # 4. Get images from Studio and plot them
    color_image, depth_image = driver.get_images()  # or just read a single image via `get_color_image`

    plt.subplot(1, 2, 1)
    plt.imshow(color_image)
    plt.subplot(1, 2, 2)
    plt.imshow(depth_image)
    plt.show()
