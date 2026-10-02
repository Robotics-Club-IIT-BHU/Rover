import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'athena_gps_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # *.rviz as well as *.yaml: athena_bench.rviz lived only in src/ and
        # was never installed, so the documented `rviz2 -d ...` command only
        # worked by reaching into the source tree.
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jashan',
    maintainer_email='thomas.j.chackenkulam295@gmail.com',
    description='GPS + VIO + IMU Nav2 navigation stack for the Athena rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixhawk_bridge = athena_gps_nav.pixhawk_bridge:main',
            'pointcloud_downsampler = athena_gps_nav.pointcloud_downsampler:main',
            'gps_waypoint_follower = athena_gps_nav.gps_waypoint_follower:main',
            'gps_waypoint_logger = athena_gps_nav.gps_waypoint_logger:main',
            'localization_monitor = athena_gps_nav.localization_monitor:main',
            'costmap_drift_check = athena_gps_nav.costmap_drift_check:main',
            'vio_gate = athena_gps_nav.vio_gate:main',
            'stack_check = athena_gps_nav.stack_check:main',
            'gps_diagnose = athena_gps_nav.gps_diagnose:main',
            'tf_check = athena_gps_nav.tf_check:main',
            'fake_gps_imu = athena_gps_nav.fake_gps_imu:main',
        ],
    },
)
