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
            'tactile_features = model_pipeline.tactile_features:main',
            'dataset_builder = model_pipeline.dataset_builder:main',
            'split_dataset = model_pipeline.split_dataset:main',
            'train = model_pipeline.train:main',
            'rollout_policy = model_pipeline.rollout_policy:main',
            'evaluate_policy = model_pipeline.evaluate_policy:main',
            'aggregate = model_pipeline.aggregate:main',
            'sampler = model_pipeline.sampler:main',
            'yolo_splitter = model_pipeline.yolo_splitter:main',
            'test_yolo = model_pipeline.test_yolo:main',
            'outlier_cleaner = model_pipeline.outlier_cleaner:main',
            'paths = model_pipeline.paths:log_all_paths',
            'create_goal_state = model_pipeline.create_goal_state:main',
            'policy_rollout = model_pipeline.policy_rollout_node:main',
            'start_task = model_pipeline.start_task:main',
            'test_kinematics = model_pipeline.kinematics_test:main',
            'save_calibration = model_pipeline.save_calibration:main',
            'test_3d_keypoint = model_pipeline.test_3d_keypoint:main',
            'visualize_rollout = model_pipeline.visualize_rollout:main',
            'create_canonical_vfe = model_pipeline.create_canonical_vfe:main',
        ],
    },
)
