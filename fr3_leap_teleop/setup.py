from setuptools import find_packages, setup

package_name = 'fr3_leap_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='maestre.paul.antoine.q8@dc.tohoku.ac.jp',
    description='Publisher node for teleoperation of Franka FR3 and LEAP Hand using DexRetargeting and Vive trackers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_vive_leap_ros2 = fr3_leap_teleop.teleop_vive_leap_ros2:run',
        ],
    },
)
