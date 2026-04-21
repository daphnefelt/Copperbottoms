from setuptools import setup

package_name = 'camera_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'line_follow',
        'line_follow_V2',
        'line_follow_daph',
        'mary_line_follow',
        'blue_follow',
        'line_detection',
        'follow_the_gap',
        'turn_right_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='copperbottoms',
    maintainer_email='sacu3661@colorado.edu',
    description='Camera navigation nodes.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'line_follow = line_follow:main',
            'line_follow_V2 = line_follow_V2:main',
            'line_follow_daph = line_follow_daph:main',
            'mary_line_follow = mary_line_follow:main',
            'blue_follow = blue_follow:main',
            'line_detection = line_detection:main',
            'follow_the_gap = follow_the_gap:main',
            'turn_right_node = turn_right_node:main',
        ],
    },
)
