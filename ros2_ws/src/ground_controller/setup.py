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
        ],
    },
)
