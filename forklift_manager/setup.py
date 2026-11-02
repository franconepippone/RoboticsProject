from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'forklift_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), 
        glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), 
        glob('launch/*.py')),
        (os.path.join('share', package_name, 'models', 'test_package'),
        glob('models/test_package/*.sdf')),
        (os.path.join('share', package_name, 'models', 'test_package'),
        glob('models/test_package/*.config')),
        (os.path.join('share', package_name, 'models', 'test_package', 'textures'),
        glob('models/test_package/textures/*.png')),
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
            'forklift_manager_node = forklift_manager.forklift_manager_node:main',
        ],
    },
)
