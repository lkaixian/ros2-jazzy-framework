from glob import glob

from setuptools import find_packages, setup

setup(
    name='nav_extension',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/nav_extension/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/nav_extension']),
        ('share/nav_extension', ['package.xml']),
        ('share/nav_extension/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Navigation BT nodes over Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nav_extension_node = nav_extension.nav_extension_node:main',
        ],
    },
)
