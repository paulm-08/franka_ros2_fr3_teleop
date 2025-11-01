import pickle
full = pickle.load(open("data/processed_datasets/dataset_joint_kpt_emb_hm.pkl","rb"))
# pick indices of trajectories you want (e.g. first 12)
subset = [full[i] for i in range(12)]
pickle.dump(subset, open("data/processed_datasets/processed_dataset_test_from_full.pkl","wb"))
