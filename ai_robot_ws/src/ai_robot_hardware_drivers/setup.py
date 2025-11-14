from setuptools import setup
from glob import glob
import os

package_name = 'ai_robot_hardware_drivers'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    url='https://github.com/api-v2-1/synapse',
    description='Hardware drivers for AI Robot (ESP8266, Kinect, Audio)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp8266_bridge_node = ai_robot_hardware_drivers.esp8266_bridge_node:main',
            'kinect_interface_node = ai_robot_hardware_drivers.kinect_interface_node:main',
            'audio_interface_node = ai_robot_hardware_drivers.audio_interface_node:main',
            'ir_sensor_node = ai_robot_hardware_drivers.ir_sensor_node:main',
        ],
    },
)
