from glob import glob
from setuptools import find_packages, setup

setup(
    name='slam', version='0.1.0', packages=find_packages(exclude=['tests', 'scripts']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/slam']),
        ('share/slam', ['package.xml']),
        ('share/slam/launch', glob('launch/*.launch.py')),
        ('share/slam/config', glob('config/*.yaml')),
        ('share/slam/rviz', glob('rviz/*.rviz')),
        ('share/slam/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    zip_safe=False,
    maintainer='Grupo TP Final PRA', maintainer_email='fortega@udesa.edu.ar',
    description='FastSLAM with independent log-odds occupancy grids',
    url='https://github.com/fortega00/TP_final',
    license='UNLICENSED',
    entry_points={'console_scripts': [
        'slam_node = slam.ros.ros_node:main',
        'toy_node = slam.ros.toy_node:main',
        'bag_tf_filter = slam.ros.tf_filter:main',
        'slam_toy = slam.toy.cli:main',
    ]},
)
