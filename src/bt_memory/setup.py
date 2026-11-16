from setuptools import find_packages, setup

setup(
    name='bt_memory',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/bt_memory']),
        ('share/bt_memory', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='WICL Task Memory: versioned storage, retrieval, cross-embodiment adapter',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
