from setuptools import find_packages, setup

package_name = 'can_comm'

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
    maintainer='naura',
    maintainer_email='nauraalwa21@gmail.com',
    description='From controller and LiDAR to CAN',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'can_translator = can_comm.can_translator:main'
            'lidar_test = can_comm.lidar_test:main'
        ],
    },
)
