import io
import os
import pickle

import cv2
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import Image as loggerImage
from stable_baselines3.common.results_plotter import load_results, ts2xy

from utils import fig2data


class SaveBestModelCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq: (int)
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """

    def __init__(self, check_freq, log_dir, verbose=1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, "best_model")
        self.env_save_path = os.path.join(log_dir, "vec_normalize.pkl")
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:

            # Retrieve training reward
            x, y = ts2xy(load_results(self.log_dir), "timesteps")
            if len(x) > 0:
                # Mean training reward over the last 100 episodes
                mean_reward = np.mean(y[-100:])
                if self.verbose > 0:
                    print(f"Num timesteps: {self.num_timesteps}")
                    print(
                        "Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(
                            self.best_mean_reward, mean_reward
                        )
                    )

                # New best model, you could save the agent here
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    # Example for saving best model
                    if self.verbose > 0:
                        print("Saving new best model at {} timesteps".format(x[-1]))
                        print(f"Saving new best model to {self.save_path}.zip")
                    self.model.save(self.save_path)
                    if hasattr(self.model.env, "save"):
                        print(f"Saving new env to {self.env_save_path}")
                        self.model.env.save(self.env_save_path)

        return True


class EvalCallback(BaseCallback):
    """
    Callback for saving the setpoint tracking plot(the check is done every ``eval_freq`` steps)

    :param eval_env: (gym.Env) The environment used for initialization
    :param n_eval_episodes: (int) The number of episodes to test the agent
    :param eval_freq: (int) Evaluate the agent every eval_freq call of the callback.
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """

    def __init__(self, eval_env, eval_freq=10000, log_dir=None, name="Eval", verbose=1):
        super().__init__(verbose)
        self.eval_env = eval_env
        self.eval_freq = eval_freq
        self.save_path = None
        self.name = name
        if log_dir is not None:
            self.log_dir = log_dir
            self.save_path = os.path.join(log_dir, "images")

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def preprocess(self, obs):
        if hasattr(self.model.env, "normalize_obs"):
            return self.model.env.normalize_obs(obs)
        else:
            return obs

    def _on_step(self) -> bool:
        if self.n_calls % self.eval_freq == 0:
            obs = self.eval_env.reset()
            obss = [obs]
            norm_obss = [self.preprocess(obs)]
            done = False
            tot_r = 0.0
            while not done:
                action, _ = self.model.predict(self.preprocess(obs), deterministic=True)
                obs, reward, done, info = self.eval_env.step(action)
                obss.append(obs)
                norm_obss.append(self.preprocess(obs))
                tot_r += reward
            obss = np.array(obss)
            norm_obss = np.array(norm_obss)
            print("Evaluation Reward: ", tot_r)

            eval_img = self.eval_env.render("rgb_array")
            self.logger.record(
                f"{self.name}/PlantOutput",
                loggerImage(eval_img, "HWC"),
                exclude=("stdout", "log", "json", "csv"),
            )
            if self.save_path is not None:
                im = Image.fromarray(eval_img)
                path = os.path.join(
                    self.save_path, f"{str(self.model.num_timesteps).zfill(7)}.png"
                )
                im.save(path)

            eval_img = self.eval_env.system.plot_gains(save=True)
            self.logger.record(
                f"{self.name}/Gains",
                loggerImage(eval_img, "HWC"),
                exclude=("stdout", "log", "json", "csv"),
            )
            if self.save_path is not None:
                im = Image.fromarray(eval_img)
                path = os.path.join(
                    self.save_path,
                    f"{str(self.model.num_timesteps).zfill(7)}_gains.png",
                )
                im.save(path)

            axis, axis_name = self.eval_env.system.get_axis()
            plt.figure(figsize=(20, 9))
            labels = self.eval_env.system.state_names.copy()
            plt.subplot(2, 1, 1)
            lineObj = plt.plot(np.arange(len(obss)), obss)
            plt.legend(lineObj, labels)
            plt.ylabel("Value")
            plt.xlabel(axis_name)
            plt.xlim(axis[0], axis[-1])
            plt.grid()

            plt.subplot(2, 1, 2)
            lineObj = plt.plot(np.arange(len(norm_obss)), norm_obss)
            plt.legend(lineObj, labels)
            plt.ylabel("Value")
            plt.xlabel(axis_name)
            plt.xlim(axis[0], axis[-1])
            plt.grid()

            plt.tight_layout()
            img = fig2data(plt.gcf())
            plt.close()
            self.logger.record(
                f"{self.name}/Inputs",
                loggerImage(img, "HWC"),
                exclude=("stdout", "log", "json", "csv"),
            )
            if self.save_path is not None:
                im = Image.fromarray(img)
                path = os.path.join(
                    self.save_path,
                    f"{str(self.model.num_timesteps).zfill(7)}_inputs.png",
                )
                im.save(path)

                obj_path = os.path.join(self.save_path, f"env_obj.pkl")
                print(f"Saving obj to {obj_path}")
                with open(obj_path, "wb") as f:
                    pickle.dump(self.eval_env.system, f)

        return True
