from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'uav_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/config', ['config/*.yaml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matteus',
    maintainer_email='victormatteus@distente.ufg.br',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_manager_node = uav_offboard.flight_manager_node:main',
            'uav_teleop_keyboard = scripts.uav_teleop_keyboard:main',
        ],
    },
)
