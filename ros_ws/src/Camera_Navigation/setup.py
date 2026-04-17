from setuptools import setup

package_name = 'camera_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['line_follow'],
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
        ],
    },
)
