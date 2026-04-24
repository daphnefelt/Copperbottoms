from setuptools import setup

package_name = 'sensor_visualization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO: Maintainer',
    maintainer_email='user@todo.todo',
    description='Sensor visualization node for vehicle speed and heading.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_visualization_node = sensor_visualization.sensor_visualization_node:main',
        ],
    },
)
