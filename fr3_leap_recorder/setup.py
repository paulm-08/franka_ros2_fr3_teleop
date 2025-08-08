from setuptools import find_packages, setup

package_name = 'fr3_leap_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='maestre.paul.antoine.q8@dc.tohoku.ac.jp',
    description='Recorder node for multimodal data (LEAP + Franka + 9DTact + RealSense)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fr3_leap_recorder = fr3_leap_recorder.fr3_leap_recorder:main',
        ],
    },
)
