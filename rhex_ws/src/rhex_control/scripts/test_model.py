import os

import numpy as np
from matplotlib import pyplot as plt


def evaluate(model, test_env, action_repeat_value, test_log_dir):
    obs = test_env.reset()
    obss = [obs]
    norm_obss = [model.env.normalize_obs(obs)]
    actions = []
    reww = []
    done = False
    tot_r = 0.0
    while not done:
        action, _ = model.predict(model.env.normalize_obs(obs), deterministic=True)
        obs, reward, done, info = test_env.step(action)
        obss.append(obs)
        norm_obss.append(model.env.normalize_obs(obs))
        actions.append(action)
        reww.append(reward)
        tot_r += reward
    print("Evaluation Reward: ", tot_r)
    test_env.render()
    fname = os.path.join(test_log_dir, "OP.png")
    plt.savefig(fname)
    test_env.system.plot_gains()
    fname = os.path.join(test_log_dir, "Actions.png")
    plt.savefig(fname)
    test_env.system.plot_actual_gains()
    fname = os.path.join(test_log_dir, "Gains.png")
    plt.savefig(fname)
    axis, axis_name = test_env.system.get_axis()
    labels = test_env.system.state_names.copy()
    obss = np.array(obss)
    norm_obss = np.array(norm_obss)

    n = len(axis[test_env.system.ksp :: action_repeat_value])
    plt.figure(figsize=(16, 4))
    lineObj = plt.plot(axis[test_env.system.ksp :: action_repeat_value], reww[:n])
    plt.legend(lineObj, labels)
    plt.ylabel("Value")
    plt.xlabel(axis_name)
    plt.xlim(axis[0], axis[-1])
    plt.grid()
    plt.title("Reward")
    fname = os.path.join(test_log_dir, "Rew.png")
    plt.savefig(fname)

    plt.figure(figsize=(16, 4))
    lineObj = plt.plot(axis[test_env.system.ksp :: action_repeat_value], actions[:n])
    plt.legend(lineObj, ["Kp", "taui", "taud"])
    plt.ylabel("Value")
    plt.xlabel(axis_name)
    plt.xlim(axis[0], axis[-1])
    plt.title("Agent Outputs")
    fname = os.path.join(test_log_dir, "agent_out.png")
    plt.savefig(fname)

    plt.figure(figsize=(16, 9))
    plt.subplot(2, 1, 1)
    lineObj = plt.plot(axis[test_env.system.ksp :: action_repeat_value], obss[:n])
    plt.legend(lineObj, labels)
    plt.ylabel("Value")
    plt.xlabel(axis_name)
    plt.xlim(axis[0], axis[-1])
    plt.grid()
    plt.title("Unnormalized States")

    plt.subplot(2, 1, 2)
    lineObj = plt.plot(axis[test_env.system.ksp :: action_repeat_value], norm_obss[:n])
    plt.legend(lineObj, labels)
    plt.ylabel("Value")
    plt.xlabel(axis_name)
    plt.xlim(axis[0], axis[-1])
    plt.grid()
    plt.title("Normalized States")

    fname = os.path.join(test_log_dir, "states.png")
    plt.savefig(fname)

    from scipy.io import savemat

    mdic = {
        "setpoint": test_env.system.r[: test_env.system.k],
        "y_rl": test_env.system.y[: test_env.system.k],
        "u_rl": test_env.system.u[: test_env.system.k],
        "gains": np.array(test_env.system.gains)[: test_env.system.k],
        "gain_components": np.array(test_env.system.gain_components)[: test_env.system.k],
        }

    fname = os.path.join(test_log_dir, "matlab_matrix.mat")
    savemat(fname, mdic)

    model.save(os.path.join(test_log_dir, "final_model"))
    import pickle

    with open(os.path.join(test_log_dir, "env_obj.pkl"), "wb") as f:
        pickle.dump(test_env, f)
    print(f"Saving output files to {test_log_dir}")
    
