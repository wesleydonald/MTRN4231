from setuptools import setup
import os
from glob import glob

package_name = 'visualization'

setup(
    name=package_name,
    version='0.0.0',
    # This now correctly finds your package folder
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install mesh files
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mtrn',
    maintainer_email='mtrn@todo.todo',
    description='Package to visualize gripper STL',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This is the corrected entry point
            'attach_gripper = visualization.attach_gripper:main',
        ],
    },
)