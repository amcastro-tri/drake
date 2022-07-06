import pickle
from pydrake.all import FindResourceOrThrow

from envs.scoop_env import ScoopEnv


if __name__ == '__main__':

    dataset = FindResourceOrThrow("drake/examples/panda/data/veggie_cylinder_tasks.pkl")
    print("= Loading tasks from", dataset)
    with open(dataset, 'rb') as f:
        task_all = pickle.load(f)

    env = ScoopEnv(
                dt=0.005,
                renders=True,
                visualize_contact=False,  # conflict with swapping geometry
                diff_ik_filter_hz=200,
                contact_solver='sap',
                panda_joint_damping=1.5,    # issue with diff ik
                )
    for ind in range(1, 100):
        task = task_all[ind]
        print('')
        print('Resetting...')
        print(f"Task - modulus: {10**task['obj_modulus']} - friction coefficient: {task['obj_mu']}")
        print('')
        obs = env.reset(task=task)
        for _ in range(1):
            action = [0, 5, 0.45]
            action[1] *= 3.1415/180
            _, _, _, info = env.step(action)
