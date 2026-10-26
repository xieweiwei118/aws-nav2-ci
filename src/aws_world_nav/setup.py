import os
from glob import glob

from setuptools import setup

package_name = 'aws_world_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Nav2 navigation on AWS RoboMaker simulation worlds with TurtleBot3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'auto_explorer = aws_world_nav.auto_explorer:main',
            'send_goals = aws_world_nav.send_goals:main',
        ],
    },
)
