from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dwe_camera'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages() will discover the 'dwe_camera' Python package
    # inside the 'dwe_camera' ROS package directory.
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=[], # This should be empty unless you have PyPI dependencies
    zip_safe=True,
    maintainer='Yuewei Fu',
    maintainer_email='yweifu@uri.edu',
    description='The dwe camera package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dwe_camera_node = dwe_camera.dwe_camera_node:main',
        ],
    },
)