from setuptools import setup, find_packages

# We install two python packages:
#   - dex_retargeting        (already lives under src/dex_retargeting)
#   - dex_retargeting_ext    (we'll create for example helpers like single_hand_detector)

setup(
    name="dex_retargeting",
    version="0.5.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    include_package_data=True,
    install_requires=[
        # Keep light; ROS apt provides numpy/scipy typically. Adjust if you want pip-driven deps.
        # "numpy>=2.0.0", "scipy>=1.11.0", "nlopt>=2.8.0", "pin>=3.3.1", "pytransform3d>=3.5.0",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dex_retargeting"]),
        ("share/dex_retargeting", ["package.xml"]),
    ],
    zip_safe=True,
)
