from glob import glob
import os

from setuptools import setup


package_name = 'cocarry_admittance_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='hungnb',
    maintainer_email='hungnb@example.com',
    description='Independent 3D admittance controller for HC10DTP co-carrying',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'admittance_controller_3d = cocarry_admittance_control.admittance_controller:main',
            'cocarry_logger = cocarry_admittance_control.cocarry_logger:main',
        ],
    },
)

