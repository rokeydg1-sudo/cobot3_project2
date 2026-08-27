from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'cobot3_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*'),
        ),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'scene_endpoint_adapter = '
            'cobot3_bringup.scene_endpoint_adapter:main',
        ],
    },
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokeydg1@gmail.com',
    description='ROS-side production integration launch structure.',
    license='MIT',
)
