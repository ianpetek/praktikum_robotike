from setuptools import setup
import os
import glob

package_name = 'yolo_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ["launch/draw_detections.launch.py"])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fpetric',
    maintainer_email='f5r1c.1m0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_processor_node = yolo_processing.yolo_processor_node:main'
        ],
    },
)
