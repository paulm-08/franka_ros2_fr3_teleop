from setuptools import setup, find_packages

package_name = "model_pipeline"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    maintainer='Maestre Paul',
    maintainer_email='maestre.paul.antoine.q8@dc.tohoku.ac.jp',
    description='ML pipeline for tactile dataset: preprocessing, training, rollout',
    license='MIT',
    install_requires=[
        "numpy",
        "opencv-python",
        "scikit-learn",
        "matplotlib",
    ],
    entry_points={
        "console_scripts": [
            "build_dataset = model_pipeline.dataset:main",
            "tactile_feature_extractor = model_pipeline.tactile_features:main",
            "train_bc = model_pipeline.train:main",
        ],
    },
)
