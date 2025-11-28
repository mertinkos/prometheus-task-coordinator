from setuptools import setup
import os
from glob import glob

package_name = 'prometheus_task_coordinator'

setup(
    name=package_name,
    version='1.0.0',
    package_dir={'': 'src'},
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
    maintainer='Mert Turkilli',
    maintainer_email='mertturkilli01@gmail.com',
    description='Task Coordinator for Prometheus Autonomous Logistics Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_coordinator = prometheus_task_coordinator.task_coordinator_node:main',
            'web_dashboard = prometheus_task_coordinator.web_dashboard_node:main',
        ],
    },
)