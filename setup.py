from setuptools import setup
import os
from glob import glob

package_name = 'den_rob_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Include package.xml and resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include launch files (if you use them)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Include config or parameter files (YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Any other data your package needs
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Digiacomo',
    maintainer_email='dominicdigiacomo16@gmail.com',
    description='Advanced ROS 2 package for Project Dentista robot software stack.',
    license='MIT',
    tests_require=['pytest'],

    # Console entry points to multiple nodes/scripts
    entry_points={
        'console_scripts': [
            'odom_publisher = den_rob_package.odom_publisher:main',
            'motor_controller = den_rob_package.motor_controller:main',
            'sensor_listener = den_rob_package.sensor_listener:main',
        ],
    },
)
