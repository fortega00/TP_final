from glob import glob
from setuptools import find_packages, setup

setup(
    name='tp_slam', version='0.1.0', packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tp_slam']),
        ('share/tp_slam', ['package.xml']),
        ('share/tp_slam/launch', glob('launch/*.launch.py')),
        ('share/tp_slam/config', glob('config/*.yaml')),
        ('share/tp_slam/rviz', glob('rviz/*.rviz')),
        ('share/tp_slam/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=False,
    maintainer='Grupo TP Final PRA', maintainer_email='fortega@udesa.edu.ar',
    description='FastSLAM with independent log-odds occupancy grids',
    url='https://github.com/fortega00/TP_final',
    license='UNLICENSED',
    entry_points={'console_scripts': [
        'slam_node = tp_slam.ros_node:main',
        'toy_node = tp_slam.toy_node:main',
        'bag_tf_filter = tp_slam.tf_filter:main',
        'slam_toy = tp_slam.cli:main',
    ]},
)
