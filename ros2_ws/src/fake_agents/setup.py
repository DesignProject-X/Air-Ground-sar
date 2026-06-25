from setuptools import find_packages, setup

package_name = 'fake_agents'

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
    description='Fake UAV and ground robot nodes for testing the SAR pipeline without real hardware',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'fake_uav_node    = fake_agents.fake_uav_node:main',
            'fake_ground_node = fake_agents.fake_ground_node:main',
        ],
    },
)
