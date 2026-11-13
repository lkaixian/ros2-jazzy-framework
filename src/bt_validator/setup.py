from glob import glob
from setuptools import find_packages, setup

setup(
    name='bt_validator',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/bt_validator/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/bt_validator']),
        ('share/bt_validator', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Static validation of BehaviorTree.CPP v4 XML',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bt_validator_node = bt_validator.validator_node:main',
            'validate_bt = bt_validator.cli:main',
        ],
    },
)
