from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=['path_planner'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tedee',
    maintainer_email='septedy44@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_node = path_planner.path_planner_node:main',
            'follower_local_planner = path_planner.follower_local_planner:main',
            'pf_visualizer_node = path_planner.pf_visualizer_node:main',
        ],
    },
)
