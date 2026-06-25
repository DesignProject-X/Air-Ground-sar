import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cf_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 注册 launch 目录
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sutd',
    maintainer_email='sutd@todo.todo',
    description='CrazyFlie controller package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cf_basic_control = cf_controller.cf_basic_control:main',
            'cf_waypoint_control = cf_controller.cf_waypoint_control:main',
            'cf_waypoint_with_scan = cf_controller.cf_waypoint_with_scan:main',
            'cf_waypoint_real = cf_controller.cf_waypoint_real:main',
        ],
    },
)