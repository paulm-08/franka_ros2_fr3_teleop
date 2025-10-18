import pickle
import numpy as np
from model_pipeline import paths
import argparse

def create_goal_from_demo(demo_path, out_path):
    with open(demo_path, 'rb') as f:
        trajectories = pickle.load(f)

    # Use the first trajectory in the file as the source for the goal
    demo_traj = trajectories[0]

    # The goal is the last state of the trajectory
    goal_tactile = demo_traj['tactile_t'][-1]
    goal_visual = demo_traj['visual_t'][-1]
    goal_joints = demo_traj['joints_t'][-1]

    # This concatenation order MUST match your dataset builder
    goal_state = np.concatenate([goal_tactile, goal_visual, goal_joints])

    with open(out_path, 'wb') as f:
        pickle.dump(goal_state, f)

    print(f"✅ Goal state created and saved to: {out_path}")
    print(f"   (Shape: {goal_state.shape})")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=str(paths.PROCESSED_DATA_DIR / 'processed_dataset2.pkl'))
    parser.add_argument("--output", default=str(paths.PROCESSED_DATA_DIR / 'insertion_goal.pkl'))
    args = parser.parse_args()
    create_goal_from_demo(args.input, args.output)

if __name__ == '__main__':
    main()