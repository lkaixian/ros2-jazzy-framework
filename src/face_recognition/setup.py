from glob import glob
from setuptools import find_packages, setup

setup(
    name='face_recognition',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/face_recognition']),
        ('share/face_recognition', ['package.xml']),
        ('share/face_recognition/launch', glob('launch/*.py')),
        ('share/face_recognition/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larm',
    maintainer_email='lkaixian91@gmail.com',
    description='YOLO11 face detection and embedding-based recognition',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'face_recognition_node = face_recognition.face_recognition_node:main',
            'auto_register_node = face_recognition.auto_register_node:main',
    'enroll_face = face_recognition.enroll_face:main',
        ],
    },
)
