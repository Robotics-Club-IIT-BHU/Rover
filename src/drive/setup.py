from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
	    ('share/ament_index/resource_index/packages',
		['resource/' + package_name]),
	    ('share/' + package_name, ['package.xml']),
	    ('share/' + package_name + '/launch',
		glob('launch/*.launch.py')),
	    ('share/' + package_name + '/urdf',
		glob('urdf/*')),
	    ('share/' + package_name + '/config',
		glob('config/*')),
	    ('share/' + package_name + '/worlds',
		glob('worlds/*')),
	],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinava',
    maintainer_email='abhinava.kalita@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = drive.move:main'
        ],
    },
)
