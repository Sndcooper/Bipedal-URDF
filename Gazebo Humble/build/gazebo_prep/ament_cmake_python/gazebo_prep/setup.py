from setuptools import find_packages
from setuptools import setup

setup(
    name='gazebo_prep',
    version='1.0.0',
    packages=find_packages(
        include=('gazebo_prep', 'gazebo_prep.*')),
)
