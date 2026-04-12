from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'person_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsh Padmalwar, Atharv Kulkarni',
    maintainer_email='hpadmalw@asu.edu, akulka96@asu.edu',
    description='Autonomous human-following robot - RAS 598',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim_person_publisher    = person_follower.sim_person_publisher:main',
            'target_tracker          = person_follower.target_tracker:main',
            'range_bearing_estimator = person_follower.range_bearing_estimator:main',
            'follow_controller       = person_follower.follow_controller:main',
            'safety_supervisor       = person_follower.safety_supervisor:main',
            'noise_injector          = person_follower.noise_injector:main',
        ],
    },
)
