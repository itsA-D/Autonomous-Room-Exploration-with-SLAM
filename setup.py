import os
from glob import glob

from setuptools import setup

package_name = 'the_mapper'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('bringup', '*.py')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ankan Debnath',
    maintainer_email='YOUR_EMAIL@example.com',
    description='ROS 2 autonomous room exploration and SLAM mapping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_exploration_node = '
            'the_mapper.autonomous_exploration_node:main',
        ],
    },
)