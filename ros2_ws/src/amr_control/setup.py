from setuptools import find_packages, setup

package_name = 'amr_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skywalker',
    maintainer_email='rokeydg1@gmail.com',
    description='World-coordinate Dolly transport mission controller for the IW Hub demo',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'amr_mission_controller = amr_control.amr_mission_controller:main',
        ],
    },
)
