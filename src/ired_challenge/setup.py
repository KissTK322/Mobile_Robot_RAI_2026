import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ired_challenge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ired',
    maintainer_email='ired@todo.todo',
    description='Industrial Logistics Challenge programs for the iRED robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_challenge = ired_challenge.navigation_node:main',
            'mps_obstacles = ired_challenge.mps_obstacles_node:main',
        ],
    },
)
