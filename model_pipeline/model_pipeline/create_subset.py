import pickle
full = pickle.load(open("/home/user/franka_ros2_ws/data/processed_datasets/processed_dataset_joint_kpt_fk_Lemb_hmL.pkl","rb"))
# pick indices of trajectories you want (e.g. first 12)
subset = [full[i] for i in range(12)]
pickle.dump(subset, open("/home/user/franka_ros2_ws/data/processed_datasets/processed_dataset_joint_kpt_fk_Lemb_hmL_test.pkl","wb"))

subset2 = [full[i] for i in range(12,len(full))]
pickle.dump(subset2, open("/home/user/franka_ros2_ws/data/processed_datasets/processed_dataset_joint_kpt_fk_Lemb_hmL_train.pkl","wb"))
