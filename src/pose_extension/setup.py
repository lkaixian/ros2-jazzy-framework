from glob import glob
from setuptools import find_packages, setup

setup(
    name='pose_extension',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/pose_extension/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/pose_extension']),
        ('share/pose_extension', ['package.xml']),
        ('share/pose_extension/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Pose/gesture BT conditions from YOLO11 pose keypoints',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_extension_node = pose_extension.pose_extension_node:main',
        ],
    },
)
