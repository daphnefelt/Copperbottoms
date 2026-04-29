from setuptools import find_packages
from setuptools import setup

setup(
    name='robo_rover',
    version='1.0.0',
    packages=find_packages(
        include=('robo_rover', 'robo_rover.*')),
)
