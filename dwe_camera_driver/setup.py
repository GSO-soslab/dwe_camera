from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dwe_camera_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'camera_parameters'), glob(os.path.join('config', 'camera_parameters', '*.yaml'))),
    ],
    install_requires=['setuptools'], 
    zip_safe=True,
    maintainer='Yuewei Fu',
    maintainer_email='yweifu@uri.edu',
    description='A ROS 2 driver for V4L2 cameras with dynamic reconfiguration and AprilTag support.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = dwe_camera_driver.camera_node:main',
            'remote_node = dwe_camera_driver.remote_node:main',
        ],
    },
)