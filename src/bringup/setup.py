import os
from glob import glob
from setuptools import setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jashan',
    maintainer_email='jashan@todo.todo',
    description='Launch files and world configuration for the UAV-guided UGV system.',
    license='MIT',
)
