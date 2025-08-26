from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tact9d'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # INSTALL SHAPE CONFIG FILE
        (os.path.join('share', package_name, 'shape_reconstruction'), 
        glob('tact9d/shape_reconstruction/shape_config*.yaml')),
        # INSTALL CALIBRATION FILES
        (os.path.join('share', package_name, 'shape_reconstruction/calibration/sensor_1/camera_calibration'),
        glob('tact9d/shape_reconstruction/calibration/sensor_1/camera_calibration/*.npy')),
        (os.path.join('share', package_name, 'shape_reconstruction/calibration/sensor_1/depth_calibration'),
        glob('tact9d/shape_reconstruction/calibration/sensor_1/depth_calibration/*.npy')),
            
        (os.path.join('share', package_name, 'shape_reconstruction/calibration/sensor_2/camera_calibration'),
        glob('tact9d/shape_reconstruction/calibration/sensor_2/camera_calibration/*.npy')),
        (os.path.join('share', package_name, 'shape_reconstruction/calibration/sensor_2/depth_calibration'),
        glob('tact9d/shape_reconstruction/calibration/sensor_2/depth_calibration/*.npy')),

        (os.path.join('share', package_name, 'shape_reconstruction/calibration/sensor_3/camera_calibration'),
        glob('tact9d/shape_reconstruction/calibration/sensor_3/camera_calibration/*.npy')),
        (os.path.join('share', package_name, 'shape_reconstruction/calibration/sensor_3/depth_calibration'),
        glob('tact9d/shape_reconstruction/calibration/sensor_3/depth_calibration/*.npy')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paul Maestre',
    maintainer_email='maestre.paul.antoine.q8@dc.tohoku.ac.jp',
    description='ROS 2 interface for tact9d sensor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = tact9d.sensor_node:main',
            'shape_reconstruction_node = tact9d.shape_reconstruction_node:main',
        ],
    },
)
