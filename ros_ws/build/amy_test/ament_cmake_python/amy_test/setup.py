from setuptools import find_packages
from setuptools import setup

setup(
    name='amy_test',
    version='0.0.1',
    packages=find_packages(
        include=('amy_test', 'amy_test.*')),
)
