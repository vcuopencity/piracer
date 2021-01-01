from setuptools import setup

package_name = 'piracer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'platform = piracer.platform:main',
        ],
    },
)
