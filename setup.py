from glob import glob
import os

from setuptools import setup

package_name = 'piracer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Morrissett',
    maintainer_email='morrissettal2@vcu.edu',
    description='ROS2 package for Waveshare PiRacer',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steering_driver = piracer.steering_driver:main',
            'throttle_driver = piracer.throttle_driver:main',
            'display_driver = piracer.display_driver:main',
            'power_monitor_driver = piracer.power_monitor_driver:main',
            'teleop_controller = piracer.teleop_controller:main',
            'ackermann_controller = piracer.ackermann_controller:main',
            'autonomy_manager = piracer.autonomy_manager:main',
            'v2x_node = piracer.v2x_node:main',
            'straight_behavior = piracer.straight_behavior:main',
            'arc_behavior = piracer.arc_behavior:main',
            'vive_driver = piracer.vive_driver:main'
        ],
    },
)
