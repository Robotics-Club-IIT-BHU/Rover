import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'athena_drive'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'firmware', 'athena_drive_fw'),
            glob('firmware/athena_drive_fw/*.ino')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jashan',
    maintainer_email='thomas.j.chackenkulam295@gmail.com',
    description='Motor drive, wiring checks and calibration for the Athena rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_bridge = athena_drive.motor_bridge:main',
            'calibrate = athena_drive.calibrate:main',
            'wiring_check = athena_drive.wiring_check:main',
            'calibrate_speed = athena_drive.calibrate_speed:main',
            'teleop = athena_drive.teleop:main',
        ],
    },
)
