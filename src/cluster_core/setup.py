from glob import glob

from setuptools import find_packages, setup

setup(
    name='cluster_core',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cluster_core']),
        ('share/cluster_core', ['package.xml']),
        ('share/cluster_core/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Agent, scheduler and supervisor: DDS job placement, failover and load balancing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cluster_agent_node = cluster_core.agent_node:main',
            'cluster_scheduler_node = cluster_core.scheduler_node:main',
            'cluster_status = cluster_core.status_cli:main',
            'plan_preview = cluster_core.plan_preview:main',
            'host_info = cluster_core.host_info_cli:main',
        ],
    },
)
