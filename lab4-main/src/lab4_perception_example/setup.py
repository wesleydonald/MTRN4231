from setuptools import find_packages, setup

package_name = 'lab4_perception_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['yolo11n-seg.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitchell',
    maintainer_email='mitch.torok@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_example = lab4_perception_example.perception_example:main'
        ],
    },
)
