import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'so_arm_100_pick_and_place'

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
    maintainer='Bruk Gebregziabher',
    maintainer_email='bruk@signalbotics.com',
    description='Hardcoded pick-and-place demo built on pymoveit2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_and_place_node = so_arm_100_pick_and_place.pick_and_place_node:main',
        ],
    },
)
