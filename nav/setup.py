from glob import glob
from setuptools import find_packages, setup

setup(
    name='nav', version='0.1.0', packages=find_packages(exclude=['tests', 'scripts']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/nav']),
        ('share/nav', ['package.xml']),
        ('share/nav/config', glob('config/*.yaml')),
        ('share/nav/launch', glob('launch/*.launch.py')),
        ('share/nav/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    zip_safe=False,
    maintainer='Grupo TP Final PRA', maintainer_email='fortega@udesa.edu.ar',
    description='Localization on a known map, planning and navigation',
    url='https://github.com/fortega00/TP_final',
    license='UNLICENSED',
    entry_points={'console_scripts': [
        'localization_node = nav.ros.localization_node:main',
        'navigator_node = nav.ros.navigator_node:main',
    ]},
)
