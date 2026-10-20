from setuptools import find_packages
from setuptools import setup

setup(
    name='lslidar_msgs',
    version='5.1.1',
    packages=find_packages(
        include=('lslidar_msgs', 'lslidar_msgs.*')),
)
