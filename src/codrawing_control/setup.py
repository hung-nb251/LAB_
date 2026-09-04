from glob import glob
import os

from setuptools import setup


package_name = 'codrawing_control'

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
    description='Independent planar admittance controller for HC10DTP co-drawing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'admittance_controller = codrawing_control.admittance_controller:main',
            'codrawing_logger = codrawing_control.codrawing_logger:main',
        ],
    },
)
