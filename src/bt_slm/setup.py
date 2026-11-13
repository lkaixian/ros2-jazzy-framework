from glob import glob
from setuptools import find_packages, setup

setup(
    name='bt_slm',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/bt_slm/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/bt_slm']),
        ('share/bt_slm', ['package.xml']),
        ('share/bt_slm/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Fine-tuned small LM (120M-320M) that turns a task spec into BT XML',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bt_slm_node = bt_slm.slm_node:main',
    'make_bt_dataset = bt_slm.dataset:main',
    'finetune_bt_slm = bt_slm.finetune:main',
        ],
    },
)
