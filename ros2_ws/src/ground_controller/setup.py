import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ground_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sv',
    maintainer_email='SylviaLuo16@users.noreply.github.com',
    description='Real TurtleBot3 ground robot controller for the SAR pipeline',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ground_controller_node = ground_controller.nav2_client_node:main',
            'coordinate_bridge_node = ground_controller.coordinate_bridge_node:main',
            'initial_pose_publisher = ground_controller.initial_pose_publisher:main',
            'map_receiver_node = ground_controller.map_receiver_node:main',
            'clock_sync_node = ground_controller.clock_sync_node:main',
            'clock_sync_client = ground_controller.clock_sync_client:main',
        ],
    },
)
