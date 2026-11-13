from glob import glob
from setuptools import find_packages, setup

setup(
    name='tts_pipeline',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/tts_pipeline/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/tts_pipeline']),
        ('share/tts_pipeline', ['package.xml']),
        ('share/tts_pipeline/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Text-to-speech: queued playback (espeak/piper) and plan announcer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tts_node = tts_pipeline.tts_node:main',
    'plan_announcer_node = tts_pipeline.plan_announcer_node:main',
        ],
    },
)
