from glob import glob
from setuptools import find_packages, setup

setup(
    name='arm_extension',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/arm_extension/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/arm_extension']),
        ('share/arm_extension', ['package.xml']),
        ('share/arm_extension/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='MoveIt-based arm BT nodes (model-agnostic: configure group/link)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_extension_node = arm_extension.arm_extension_node:main',
        ],
    },
)
