from setuptools import find_packages, setup

setup(
    name='nav_sim',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/nav_sim']),
        ('share/nav_sim', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Tiny 2D simulator for testing Nav2/SLAM without a robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nav_sim_node = nav_sim.nav_sim_node:main',
            'make_test_map = nav_sim.make_test_map:main',
        ],
    },
)
