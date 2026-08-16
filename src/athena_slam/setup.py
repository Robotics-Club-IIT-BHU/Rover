from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'athena_slam'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # install yaml configs (e.g. CloudtoCost.yaml) so params_file:= can find them
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='thomas.j.chackenkulam295@gmail.com',
    description='SLAM package for Athena rover using RTAB-Map and RealSense',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "filter = athena_slam.point_cloud_filter:main",
            "coordextract = athena_slam.coord_extract:main",
            "vispoint = athena_slam.pointcloudvis:main",
            "simlin = athena_slam.linearSim:main"
        ],
    },
)
