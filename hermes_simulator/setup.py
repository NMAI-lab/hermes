import os
from setuptools import setup
from glob import glob

package_name = 'hermes_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['config/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bardibloo',
    maintainer_email='bardiaparmoun@gmail.com',
    description='Handles the sensor interpretation and the robot control of the simulator.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_sensor = hermes_simulator.perceptions.lidar_sensor:main',
            'beacon_sensor = hermes_simulator.perceptions.beacon_sensor:main'
        ],
    },
)
