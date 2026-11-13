from glob import glob
from setuptools import find_packages, setup

setup(
    name='pid_extension',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pid_extension']),
        ('share/pid_extension', ['package.xml']),
        ('share/pid_extension/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='pid extension: BT node manifest and executor hook',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pid_extension_node = pid_extension.pid_extension_node:main',
        ],
    },
)
