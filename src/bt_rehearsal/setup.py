from setuptools import find_packages, setup

setup(
    name='bt_rehearsal',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/bt_rehearsal']),
        ('share/bt_rehearsal', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='WICL rehearsal loop and simulator interface',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'wicl = bt_rehearsal.cli:main',
        'rehearsal_node = bt_rehearsal.rehearsal_node:main',
    ]},
)
