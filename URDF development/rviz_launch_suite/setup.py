from glob import glob
import os

from setuptools import setup


package_name = 'biped_rviz_launch_suite'


def _tree_data_files(root_dir, install_root):
    data_files = []
    for dirpath, _, filenames in os.walk(root_dir):
        if not filenames:
            continue
        relative_dir = os.path.relpath(dirpath, root_dir)
        target_dir = os.path.join('share', package_name, install_root)
        if relative_dir != '.':
            target_dir = os.path.join(target_dir, relative_dir)
        data_files.append((target_dir, [os.path.join(dirpath, filename) for filename in filenames]))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + _tree_data_files('assets', 'assets'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='Launch files and copied URDF assets for RViz viewing.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'five_bar_state_publisher = biped_rviz_launch_suite.five_bar_state_publisher:main',
        ],
    },
)