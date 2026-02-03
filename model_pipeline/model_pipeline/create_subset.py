import pickle
full = pickle.load(open("data/processed_datasets/processed_dataset_task_3dkpt_fk_Lemb_hm.pkl","rb"))
# pick indices of trajectories you want (e.g. first 12)
subset = [full[i] for i in range(12)]
subset2 = [full[i] for i in range(12,len(full))]
pickle.dump(subset, open("data/processed_datasets/processed_dataset_task_3dkpt_fk_Lemb_hm_test.pkl","wb"))
pickle.dump(subset2, open("data/processed_datasets/processed_dataset_task_3dkpt_fk_Lemb_hm_train.pkl","wb"))
