from glob import glob
from setuptools import find_packages, setup

setup(
    name='odom_extension',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/odom_extension/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/odom_extension']),
        ('share/odom_extension', ['package.xml']),
        ('share/odom_extension/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Odometry-based motion BT nodes (NavigateTo, DriveDistance, RotateBy)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_extension_node = odom_extension.odom_extension_node:main',
        ],
    },
)
