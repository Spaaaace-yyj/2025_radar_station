from setuptools import find_packages
from setuptools import setup

setup(
    name='livox_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('livox_interfaces', 'livox_interfaces.*')),
)
