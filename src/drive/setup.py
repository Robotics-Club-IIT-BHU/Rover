import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Pico firmware kept with the package for reference/versioning
        (os.path.join('share', package_name, 'firmware', 'sketch_feb10a'),
            glob('firmware/sketch_feb10a/*.ino')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='thomas.j.chackenkulam295@gmail.com',
    description='Athena rover drive: differential PWM serial bridge for the '
                'Pico W motor controller (v2 firmware) + keyboard teleop',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move = drive.move:main',
            'teleop = drive.teleop:main'
        ],
    },
)
