from setuptools import setup, find_packages

package_name = 'denstista_ros2_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='.'),
    package_dir={'': '.'},
    # Include Python files in package
    include_package_data=True,
    # package_data only for non-launch files (optional)
    package_data={
        package_name: ['*.py'],  # or your python modules if any
    },
    # This is the key part: explicitly install launch files in the share folder
    data_files=[
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/slam_with_rplidar.launch.py']),
    ('share/' + package_name + '/scripts', ['scripts/create_map_dir.sh']),
    ('share/' + package_name + '/config', ['config/slam_toolbox.yaml']),
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
            'OmniSerialBridge= denstista_ros2_drive.OmniSerialBridge:main',
        ],
    },
)
