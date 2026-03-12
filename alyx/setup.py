from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'alyx'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Include model files (URDF, Gazebo, etc.)
        (os.path.join('share', package_name, 'model'), glob('model/*')),
        
        # Include mesh files
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubh',
    maintainer_email='shubh06kesar@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'swerve = alyx.swerve:main',
        ],
    },
)
