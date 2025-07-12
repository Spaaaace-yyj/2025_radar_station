from setuptools import find_packages
from setuptools import setup

setup(
    name='radar_station_interface',
    version='0.0.0',
    packages=find_packages(
        include=('radar_station_interface', 'radar_station_interface.*')),
)
