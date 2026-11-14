from setuptools import find_packages, setup
setup(
    name='yolo11_common',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/yolo11_common']),
        ('share/yolo11_common', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='Shared YOLO11 helpers: image source and model loading',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
