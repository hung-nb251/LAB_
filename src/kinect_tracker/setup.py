from setuptools import find_packages, setup

package_name = 'kinect_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/pose_landmarker_full.task']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hungnb',
    maintainer_email='hungnb@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_node = kinect_tracker.kinect_node:main'
        ],
    },
)
