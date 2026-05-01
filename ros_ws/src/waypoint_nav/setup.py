from setuptools import find_packages, setup

package_name = 'waypoint_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/waypoint_nav_launch.py']),
        ('share/' + package_name, ['nav2_params.yaml', 'run_waypoints.py', 'map_relay.py', 'pose_history_save.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='copperbottoms',
    maintainer_email='copperbottom@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)