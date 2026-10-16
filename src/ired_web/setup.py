import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ired_web'


def tree(src):
    """data_files entries for every file under src, keeping sub-folders."""
    out = []
    for root, _dirs, files in os.walk(src):
        if files:
            out.append((os.path.join('share', package_name, root), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + tree('web'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ired',
    maintainer_email='ired@todo.todo',
    description='Phone/tablet web app for the iRED robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'manager = ired_web.manager:main',
        ],
    },
)
