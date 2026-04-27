from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wildbridge_mavros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Edouard Rolland',
    maintainer_email='edr@mmmi.sdu.dk',
    description='MAVROS-compatible ROS 2 interface for WildBridge DJI drone control',
    license='MIT',
    scripts=['scripts/auto_discover_launch.sh'],
    entry_points={
        'console_scripts': [
            'mavros_bridge = wildbridge_mavros.mavros_bridge:main',
            'auto_launch = wildbridge_mavros.auto_launch:main',
            'auto_mavros_bridge = wildbridge_mavros.auto_mavros_bridge:main',
        ],
    },
)
