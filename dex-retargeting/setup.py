from setuptools import setup, find_packages

setup(
    name="dex_retargeting",
    version="0.5.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    include_package_data=True,  # important for MANIFEST.in to work
    package_data={
        "dex_retargeting": ["assets/**/*"],  # include all files recursively
    },
    install_requires=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dex_retargeting"]),
        ("share/dex_retargeting", ["package.xml"]),
    ],
)
zip_safe=False,