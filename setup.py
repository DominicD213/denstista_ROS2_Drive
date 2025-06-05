from setuptools import setup, find_packages

package_name = 'denstista_ros2_drive'

setup(
     name=package_name,
    version='0.0.0',
    packages=find_packages(where='.'),  # Automatically find the nested package
    package_dir={'': '.'},              # Current dir is the root
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_with_rplidar.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for denstista drive',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = denstista_ros2_drive.odom_publisher:main',
            'motor_controller = denstista_ros2_drive.motor_controller:main',
        ],
    },
)
