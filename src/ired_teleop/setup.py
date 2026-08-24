from setuptools import find_packages, setup

package_name = 'ired_teleop'

setup(
    name=package_name,
    version='2.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='AIMs Lab, KMITL',
    author_email='aimslabkmitl@gmail.com',
    maintainer='T. Thepsit',
    maintainer_email='t.thepsit@gmail.com',
    description='iRED Teleop : This package is used for command to move the iRED robot.',
    license='Apache-2.0',
    keywords=['ROS'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = ired_teleop.script.teleop_keyboard:main'
        ],
    },
)
