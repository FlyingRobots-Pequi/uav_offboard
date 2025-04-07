from setuptools import find_packages, setup

package_name = 'hermit_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/goto_setpoints.yaml']),
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
            'fase_1 = hermit_offboard.fase_1:main',
            'fase_2 = hermit_offboard.fase_2:main',
            'fase_3 = hermit_offboard.fase_3:main',
            'tdp = hermit_offboard.tdp:main',
            'line_trajectory = hermit_offboard.line_trajectory:main',
            'drone_controller = hermit_offboard.drone_controller:main',
            'takeoff_and_landing = hermit_offboard.takeoff_and_landing:main',
            'register_setpoints = scripts.register_setpoints:main',
        ],
    },
)
