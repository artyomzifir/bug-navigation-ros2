from setuptools import setup
from glob import glob

package_name = 'bug_navigation'

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
    maintainer='student',
    maintainer_email='student@example.com',
    description='Bug 0, Bug 1 and Tangent Bug planners',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bug_planner_node = bug_navigation.bug_planner:main',
            'bug0_planner_node = bug_navigation.bug0_planner:main',
            'bug1_planner_node = bug_navigation.bug1_planner:main',
        ],
    },
)
