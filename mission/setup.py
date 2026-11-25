from glob import glob
from setuptools import find_packages, setup

setup(
    name='mission', version='0.1.0', packages=find_packages(exclude=['tests', 'scripts']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mission']),
        ('share/mission', ['package.xml']),
        ('share/mission/config', glob('config/*.yaml')),
        ('share/mission/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    zip_safe=False,
    maintainer='Grupo TP Final PRA', maintainer_email='fortega@udesa.edu.ar',
    description='Autonomous exploration and red cone search',
    url='https://github.com/fortega00/TP_final',
    license='UNLICENSED',
    entry_points={'console_scripts': [
        'camera_throttle_node = mission.ros.camera_throttle_node:main',
        'mission_node = mission.ros.mission_node:main',
    ]},
)
