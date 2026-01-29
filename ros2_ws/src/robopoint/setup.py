from setuptools import find_packages, setup

package_name = 'robopoint'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robopoint.launch.py', 'launch/robopoint_system.launch.py']),
    ],    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='Lesterliou@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'grasp_node = robopoint.grasp_node:main',
        ],
    },
)
