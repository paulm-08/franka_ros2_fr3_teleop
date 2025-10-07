from setuptools import setup

package_name = 'model_pipeline'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maestre Paul',
    maintainer_email='maestre.paul.antoine.q8@dc.tohoku.ac.jp',
    description='ML pipeline for tactile dataset: preprocessing, training, rollout',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'build_dataset = model_pipeline.build_dataset:main',
            'tactile_feature_extractor = model_pipeline.tactile_features:main',
            'train_bc = model_pipeline.train:main',
        ],
    },
)
