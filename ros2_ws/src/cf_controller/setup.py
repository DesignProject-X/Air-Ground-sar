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
            'cf_waypoint_real = cf_controller.cf_waypoint_real:main',
            'cf_mission_node = cf_controller.cf_mission_node:main',
            'cf_hover_real = cf_controller.cf_hover_real:main',
            'cf_hover_real_continue = cf_controller.cf_hover_real_continue:main',
            'cf_hover_sim = cf_controller.cf_hover_sim:main',
        ],
    },
)