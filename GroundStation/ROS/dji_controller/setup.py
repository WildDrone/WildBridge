from setuptools import find_packages, setup

package_name = 'dji_controller'
submodules = 'dji_controller/submodules'

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
    maintainer='edr',
    maintainer_email='65714311+edouardrolland@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dji_node = dji_controller.controller:main'
        ],
    },
)
