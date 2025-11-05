from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_gripper_pkg'



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line includes your launch file
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'pyserial'], # Make sure pyserial is here
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package to control an Arduino gripper',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This line assumes you are using the service-based node
            'end_effector_node = my_gripper_pkg.end_effector_node:main',
            'arduino_serial_node = my_gripper_pkg.arduino_serial_node:main',
        ],
    },
)