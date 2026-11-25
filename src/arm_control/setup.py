from setuptools import find_packages, setup

setup(
    name='arm_control',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/arm_control']),
        ('share/arm_control', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Guarded arm motion helpers',
    license='Apache-2.0',
)
