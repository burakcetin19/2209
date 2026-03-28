import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aircraft_taxi_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'),
            glob('models/*') + ['models/.gitkeep']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='burak',
    maintainer_email='burak@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'base_controller = aircraft_taxi_control.base_controller:main',
            'line_follower = aircraft_taxi_control.line_follower:main',
        ],
    },
)
