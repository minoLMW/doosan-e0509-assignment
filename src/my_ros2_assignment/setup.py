from setuptools import find_packages, setup

package_name = 'my_ros2_assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mino',
    maintainer_email='mino@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = my_ros2_assignment.my_node:main',
            'movegroup_client_min = my_ros2_assignment.movegroup_client_min:main',
            'movegroup_client_xyz = my_ros2_assignment.movegroup_client_xyz:main',
            'movegroup_sequence = my_ros2_assignment.movegroup_sequence:main',
            'assignment_status_node = my_ros2_assignment.assignment_status_node:main',
            'stop_client = my_ros2_assignment.stop_client:main',
        ],
    },
)
