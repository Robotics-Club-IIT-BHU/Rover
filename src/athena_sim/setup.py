import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'athena_sim'

# Helper to collect all files in a folder tree for data_files
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f])
          for f in package_files('worlds')],
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f])
          for f in package_files('models')],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jashan',
    maintainer_email='jashanpreetsinghbrar01@gmail.com',
    description='Simulation harness for Athena rover in Gazebo Fortress (IRC 2027)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_gps_rtk = athena_sim.sim_gps_rtk:main',
            'vio_sim = athena_sim.vio_sim:main',
            'mission_runner = athena_sim.mission_runner:main',
            'generate_terrain = athena_sim.generate_terrain:main',
            'briefing_tool = athena_sim.briefing_tool:main',
            'render_map = athena_sim.render_map:main',
            'check_env = athena_sim.check_env:main',
        ],
    },
)
