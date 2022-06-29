import pickle
from pydrake.all import FindResourceOrThrow

from envs.push_traj_env import PushTrajEnv


if __name__ == '__main__':
    dataset = FindResourceOrThrow("drake/examples/panda/data/bottle_tasks.pkl")
    print("= Loading tasks from", dataset)
    with open(dataset, 'rb') as f:
        task_all = pickle.load(f)
    env = PushTrajEnv(
                    dt=0.002,
                    renders=True,
                    visualize_contact=False,
                    diff_ik_filter_hz=-1,
                    )
    for ind in range(100):
        task = task_all[ind]
        print('Resetting...')
        print(f"Task - modulus: {10**task['obj_modulus']} - friction coefficient: {task['obj_mu']}")
        obs = env.reset(task=task)
        for _ in range(1):
            action = [0, 0, 0]
            _, _, _, info = env.step(action)
