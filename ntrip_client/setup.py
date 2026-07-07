import os
from setuptools import setup, find_packages

package_name = 'ntrip_client'

setup(
    name=package_name,
    version='3.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Austin Johnson',
    maintainer_email='austin.johnson@anellophotonics.com',
    keywords=['ROS2', 'NTRIP', 'GNSS'],
    description='NTRIP client for the ANELLO ROS2 driver',
    license='MIT License',
    # Deprecated in setuptools, but colcon-core reads it to select the
    # pytest runner for this package — without it `colcon test` falls
    # back to unittest and silently runs zero tests.
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ntrip_ros = ntrip_client.ntrip_ros:main',
        ],
    },
)
