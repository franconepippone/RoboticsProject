from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'robot'


models_files = []

for root, dirs, files in os.walk('models'):
    if not files:
        continue

    relative_dir = os.path.relpath(root, 'models')
    destination = os.path.join('share', package_name, 'models')

    if relative_dir != '.':
        destination = os.path.join(destination, relative_dir)

    models_files.append((
        destination,
        [os.path.join(root, filename) for filename in files]
    ))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'description/'),
         glob('description/*')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*')),
        (os.path.join('share', package_name, 'map'),
         glob('map/*')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.*')),
         *models_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'robot_joy_control = robot.robot_joy_control:main',
        ],
    },
)
