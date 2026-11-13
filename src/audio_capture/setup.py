from glob import glob
from setuptools import find_packages, setup

setup(
    name='audio_capture',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/audio_capture/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/audio_capture']),
        ('share/audio_capture', ['package.xml']),
        ('share/audio_capture/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Microphone (arecord) or WAV-file source publishing AudioChunk',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'audio_capture_node = audio_capture.audio_capture_node:main',
        ],
    },
)
