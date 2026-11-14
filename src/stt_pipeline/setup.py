from glob import glob
from setuptools import find_packages, setup

setup(
    name='stt_pipeline',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/stt_pipeline/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/stt_pipeline']),
        ('share/stt_pipeline', ['package.xml']),
        ('share/stt_pipeline/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Speech-to-text: VAD segmentation, pluggable engines, task gating',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vad_node = stt_pipeline.vad_node:main',
    'stt_node = stt_pipeline.stt_node:main',
    'sound_event_node = stt_pipeline.sound_event_node:main',
    'transcript_to_task_node = stt_pipeline.transcript_to_task_node:main',
        ],
    },
)
