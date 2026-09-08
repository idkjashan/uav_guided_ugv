from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'road_survey'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jashan',
    maintainer_email='jashanpreetsinghbrar01@gmail.com',
    description='UAV road survey: depth -> 2.5-D elevation -> road costmap.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terrain_mapper = road_survey.terrain_mapper_node:main',
            'map_publisher = road_survey.map_publisher_node:main',
            'height_slicer = road_survey.height_slicer_node:main',
            'tune_offline = road_survey.tune_offline:main',
        ],
    },
)
