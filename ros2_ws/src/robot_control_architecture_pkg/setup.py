from setuptools import find_packages, setup

package_name = 'robot_control_architecture_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/launch', ['launch/approach_person_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aimalohi Alakhume',
    maintainer_email='aoa36@cornell.edu',
    description='This package implements a deliberative/hybrid robot control architecture to enable a robot to approach a person while avoiding obstacles.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_architecture_node = robot_control_architecture_pkg.robot_control_architecture_node:main',
            'deliberative_architecture_node = robot_control_architecture_pkg.deliberative_architecture_node:main'
        ],
    },
)

