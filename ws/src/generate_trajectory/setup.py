from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'generate_trajectory'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # LAUNCH FILES !!!
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='linda',
    maintainer_email='linda.bozan5@gmail.com',

    description='Generate trajectory + YOLO + ArUco navigation',

    license='TODO',

    entry_points={
        'console_scripts': [
            'generate_path_node = generate_trajectory.generate_path_node:main',
        ],
    },
)

# ros2 launch generate_trajectory connect_objects.launch.py
