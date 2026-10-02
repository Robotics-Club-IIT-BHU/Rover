import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'athena_remote'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jashan',
    maintainer_email='thomas.j.chackenkulam295@gmail.com',
    description='Operator front end for Athena: Foxglove, goals, waypoints, remote access',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'captive_login = athena_remote.captive_login:main',
            'goal_manager = athena_remote.goal_manager:main',
            'waypoint_manager = athena_remote.waypoint_manager:main',
            'trajectory = athena_remote.trajectory:main',
            'teleop_mux = athena_remote.teleop_mux:main',
            'nav_status = athena_remote.nav_status:main',
            'panel_camera = athena_remote.panel_camera:main',
            'panel_cloud = athena_remote.panel_cloud:main',
        ],
    },
)
