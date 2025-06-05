from setuptools import setup
from glob import glob
import os

package_name = 'denstista_ros2_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Digiacomo',
    maintainer_email='dominicdigiacomo16@gmail.com',
    description='Advanced ROS 2 package for Project Dentista robot software stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = denstista_ros2_drive.odom_publisher:main',
            'motor_controller = denstista_ros2_drive.motor_controller:main',
        ],
    },
)
