from setuptools import find_packages, setup

package_name = 'stop_sign'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/stop_sign_detect', ['stop_sign_detect/cascade_stop_sign.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='copperbottoms',
    maintainer_email='copperbottom',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stop_sign_detect = stop_sign_detect.stop_sign_detect:main'
        ],
    },
)
