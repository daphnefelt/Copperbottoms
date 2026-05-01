from setuptools import find_packages, setup

package_name = 'realsense_publisher'

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
    maintainer='copperbottoms',
    maintainer_email='sacu3661@colorado.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'realsense_node = realsense_publisher.realsense_node:main',
		'realsense_depth_node = realsense_publisher.realsense_depth_node:main',
        ],
    },
)
