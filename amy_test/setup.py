from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'amy_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='copperbottoms',
    maintainer_email='copperbottoms@gmail.com',
    description='Launch file package for full rover system with LIO odometry',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lio_odometry = amy_test.Slam_landmark.odometery.LIO:main',
            'odometry_monitor = amy_test.verification.odometry_monitor:main',
            # Line following
            'line_follow_amy = amy_test.line_follow_amy:main',
            # Testing & verification
            'tape_contour_rt = amy_test.verification.tape_contour_rt:main',
            # Latency monitoring infrastructure
            'topic_rate_monitor = amy_test.topic_rate_monitor:main',
            'vision_latency_monitor = amy_test.vision_latency_monitor:main',
            'motor_latency_monitor = amy_test.motor_latency_monitor:main',
            'latency_aggregator = amy_test.latency_aggregator:main',
            'performance_logger = amy_test.performance_logger:main',
            'latency_dashboard = amy_test.latency_dashboard:main',
            'automated_latency_test = amy_test.automated_latency_test:main',
        ],
    },
)
