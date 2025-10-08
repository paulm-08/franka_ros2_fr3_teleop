from setuptools import setup, find_packages

package_name = "model_pipeline"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    maintainer='Maestre Paul',
    maintainer_email='maestre.paul.antoine.q8@dc.tohoku.ac.jp',
    description='ML pipeline for tactile dataset: preprocessing, training, rollout',
    license='MIT',
    install_requires=[
        "numpy",
        "opencv-python",
        "scikit-learn",
        "matplotlib",
        "setuptools",
    ],
    entry_points={
        'console_scripts': [
            'dataset_builder = model_pipeline.dataset_builder:main',
            'split_dataset = model_pipeline.split_dataset:main',
            'train = model_pipeline.train:main',
            'rollout_policy = model_pipeline.rollout_policy:main',
            'evaluate_policy = model_pipeline.evaluate_policy:main',
        ],
    },
)
