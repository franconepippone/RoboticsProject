from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pallet_pickup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.*')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='americo',
    maintainer_email='americo.cherubini2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'laser_obstacle_detector = pallet_pickup.laser_obstacle_detector:main',
        'dock_pose_bridge = pallet_pickup.dock_pose_bridge_node:main',
        'pickup_server = pallet_pickup.pickup_server_node:main',
        'odometric_controller = pallet_pickup.control_utils.odometric_controller_node:main',
    ],
},
)
