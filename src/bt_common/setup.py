from glob import glob
from setuptools import find_packages, setup

setup(
    name='bt_common',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/bt_common']),
        ('share/bt_common', ['package.xml']),
        ('share/bt_common/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='BT node catalog, extension executor base and cmd_vel mux',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_mux_node = bt_common.cmd_vel_mux:main',
            'battery_guard_node = bt_common.battery_guard:main',
            'scan_mask_node = bt_common.scan_mask:main',
        ],
    },
)
