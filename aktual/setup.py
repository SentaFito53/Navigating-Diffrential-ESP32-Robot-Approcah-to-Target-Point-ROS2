import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aktual'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', 'aktual', 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senta',
    maintainer_email='senta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strategy = aktual.strategy:main',
            'vision = aktual.detection:main',
            'communication = aktual.communication:main',
            'diff_drive = aktual.diff_drive:main',
            'get_robot_pos_service = aktual.get_robot_pos_service:main',
            'get_target_pos_service = aktual.get_target_pos_service:main',
        ],
    },
)

