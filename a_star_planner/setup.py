from setuptools import setup
from glob import glob

package_name = 'a_star_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='0',
    maintainer_email='@',
    description='A* global planner + path follower',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'a_star_planner_node = a_star_planner.a_star_planner_node:main',
            'path_follower_node = a_star_planner.path_follower_node:main',
        ],
    },
)
