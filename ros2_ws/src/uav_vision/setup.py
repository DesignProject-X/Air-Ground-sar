from setuptools import find_packages, setup

package_name = 'uav_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sutd',
    maintainer_email='sutd@todo.todo',
    description='Real target detection (YOLO + ArUco) on the UAV camera stream',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'target_detector_node = uav_vision.target_detector_node:main',
        ],
    },
)
