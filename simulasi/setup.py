from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simulasi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Instalasi resource dan package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Instalasi file launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senta',
    maintainer_email='senta@todo.todo',
    description='Launch system for simulation and vision/control nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              # Jika kamu nanti punya script node Python, masukkan di sini
              'gazebo_vision = simulasi.gazebo_vision:main',
              'gazebo_control = simulasi.control_robot:main',
        ],
    },
)

