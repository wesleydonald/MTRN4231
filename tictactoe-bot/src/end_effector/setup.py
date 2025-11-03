from setuptools import find_packages, setup

package_name = 'end_effector'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],  # pyserial is needed for Arduino comm
    zip_safe=True,
    maintainer='mtrn',
    maintainer_email='wesleydonaldnz@gmail.com',
    description='ROS 2 Python node to control the gripper via Arduino.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper = end_effector.gripper:main',
        ],
    },
)