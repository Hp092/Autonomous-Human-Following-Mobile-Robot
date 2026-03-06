from setuptools import setup

package_name = 'person_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/person_follower.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@example.com',
    description='Starter ROS 2 package skeleton (Milestone 1 tech requirement).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower_node = person_follower.follower_node:main',
        ],
    },
)
