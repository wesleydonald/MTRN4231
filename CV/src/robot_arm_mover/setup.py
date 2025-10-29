import os
from glob import glob
from setuptools import setup

package_name = 'robot_arm_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sherry',
    maintainer_email='guoshanrui07@gmail.com',
    description='A package to move a robot arm to a detected piece using MoveIt 2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'robot_arm_mover = robot_arm_mover.arm:main',
        ],
    },
)