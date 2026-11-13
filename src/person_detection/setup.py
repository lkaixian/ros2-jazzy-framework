from glob import glob
from setuptools import find_packages, setup

setup(
    name='person_detection',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/person_detection']),
        ('share/person_detection', ['package.xml']),
        ('share/person_detection/launch', glob('launch/*.py')),
        ('share/person_detection/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='YOLO11 person detection, tracking and pose',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'person_detection_node = person_detection.person_detection_node:main',
            'person_depth_node = person_detection.person_depth_node:main',
        ],
    },
)
