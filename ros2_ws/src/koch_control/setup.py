import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'koch_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install scripts
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='Lesterliou@gmail.com',
    description='Koch Robot Control Package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'koch_leader_control = koch_control.koch_leader_control:main',
            'koch_follower_control = koch_control.koch_follower_control:main',
            'koch_leader_follower_control = koch_control.koch_leader_follower_control:main',
            'koch_teleop_bridge = koch_control.koch_teleop_bridge:main',
            'koch_calibration = koch_control.koch_calibration:main',
        ],
    },
)
