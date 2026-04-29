from setuptools import setup

package_name = 'visualize_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO: Maintainer',
    maintainer_email='user@todo.todo',
    description='Telemetry visualization node for vehicle sensors.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualize_sensors = visualize_sensors.visualize_sensors:main',
        ],
    },
)
