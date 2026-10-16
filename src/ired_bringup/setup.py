from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'ired_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ired',
    maintainer_email='ired@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	    'modbus_serial = ired_bringup.modbus_serial:main',
	    'forward_kinematics = ired_bringup.forward_kinematics:main',
            'inverse_kinematics = ired_bringup.inverse_kinematics:main',
	    'imu_node = ired_bringup.imu:main',
	    'odom_node = ired_bringup.odom:main'
        ],
    },
)
