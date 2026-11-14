from glob import glob
from setuptools import find_packages, setup

setup(
    name='bt_orchestrator',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/bt_orchestrator/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/bt_orchestrator']),
        ('share/bt_orchestrator', ['package.xml']),
        ('share/bt_orchestrator/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Runs task -> llama.cpp -> SLM -> validator with feedback retries',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'orchestrator_node = bt_orchestrator.orchestrator_node:main',
            'robot_mode_node = bt_orchestrator.robot_mode_node:main',
    'scene_summarizer_node = bt_orchestrator.scene_summarizer_node:main',
            'submit_bt = bt_orchestrator.submit_bt:main',
        ],
    },
)
