from argparse import ArgumentParser

import matplotlib.pyplot as plt

from jacobi import Camera, Frame, Intrinsics, Studio
from jacobi_vision.images import ColorImage
from jacobi_vision.drivers import RealsenseCameraDriver


if __name__ == '__main__':
    parser = ArgumentParser('Using the Jacobi RealSense driver.')
    parser.add_argument('--studio', action='store_true', help='Show data in Studio')
    parser.add_argument('--loop', action='store_true', help='Loop for a live stream')

    args = parser.parse_args()

    # 1. Add camera to empty Studio project
    intrinsics = Intrinsics(500.0, 500.0, optical_center_x=320, optical_center_y=240, width=640, height=480)
    camera = Camera('model', 'Camera', Frame(z=2.0, a=3.1415), intrinsics)

    studio = Studio()
    studio.reset()
    studio.add_camera(camera)

    # 2. Setup the Intel Realsense RGBD camera
    driver = RealsenseCameraDriver(camera)

    # 3. Get and visualize live sensor data
    if args.studio:
        while args.loop:
            image = ColorImage(driver.get_color_image())
            studio.set_camera_image_encoded(image.encode(), camera)

    else:
        color_image, depth_image = driver.get_images()

        plt.subplot(1, 2, 1)
        plt.imshow(color_image)
        plt.subplot(1, 2, 2)
        plt.imshow(depth_image, cmap='jet')
        plt.show()
