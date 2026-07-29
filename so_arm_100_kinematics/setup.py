from setuptools import find_packages, setup

package_name = 'so_arm_100_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tonoward',
    maintainer_email='tono_war@hotmail.com',
    description=(
        'Closed-form FK/IK for the SO-100 5-DOF arm chain. Pure Python, '
        'zero ROS dependency -- meant to be vendored verbatim into the '
        'Blender addon.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
)
