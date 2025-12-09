from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'diffrobot_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erdemdavaa',
    maintainer_email='mherodavaa@gmail.com',
    description='Navigation2 setup for differential robot.',
    license='Apache-2.0',
    entry_points={},
)
