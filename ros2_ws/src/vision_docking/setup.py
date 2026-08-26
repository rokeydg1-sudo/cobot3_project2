from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'vision_docking'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*'),
        ),
        (
            os.path.join('share', package_name, 'models'),
            glob('models/*'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='rokey',
    maintainer_email='rokey9D1@gmail.com',
    description='Vision-based safe Dolly docking runtime for IW Hub.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'dolly_docking_node = vision_docking.dolly_docking_node:main',
        ],
    },
)
