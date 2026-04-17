import os
from glob import glob
from setuptools import setup

package_name = 'rtabmap_diff_drive_tutorial'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name], # Although we don't have python modules in this pkg, this is needed
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        # Install RViz config files
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        # Install world files (Make sure test.world is in the worlds/ directory)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',             # TODO: Update maintainer
    maintainer_email='your_email@example.com', # TODO: Update maintainer email
    description='RTAB-Map tutorial with differential drive robot in Gazebo (ROS 2 Humble)',
    license='Apache License 2.0',   # Or your preferred license
    entry_points={
        'console_scripts': [
            # Add executables here if you create python nodes later
        ],
    },
)
