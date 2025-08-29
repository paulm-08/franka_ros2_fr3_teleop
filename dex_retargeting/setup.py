from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dex_retargeting'

def package_files(directory):
    # Get all files recursively, with paths relative to the package root
    paths = []
    for path in glob(os.path.join(directory, '**', '*'), recursive=True):
        if os.path.isfile(path):
            # This will be e.g. 'assets/file.txt' or 'assets/subdir/file.txt'
            paths.append(path)
    return paths

# Gather all data files, preserving their relative paths
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add all files under assets and configs, preserving their relative paths
for folder in ['assets', 'configs']:
    for file in package_files(folder):
        # file is e.g. 'assets/foo/bar.txt'
        data_files.append(
            (os.path.join('share', package_name, os.path.dirname(file)), [file])
        )

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paul Maestre',
    maintainer_email='maestre.paul.antoine.q8@dc.tohoku.ac.jp',
    description='dex_based hand retargeting for robotic manipulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your scripts here if needed
        ],
    },
)
