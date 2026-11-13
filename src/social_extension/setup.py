from glob import glob
from setuptools import find_packages, setup

setup(
    name='social_extension',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/social_extension/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/social_extension']),
        ('share/social_extension', ['package.xml']),
        ('share/social_extension/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Speak and face-enrol BT nodes (TTS + face gallery)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'social_extension_node = social_extension.social_extension_node:main',
        ],
    },
)
