#!/usr/bin/env python3

import rclpy
from miscellaneous import constrain_angle
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
import rclpy.parameter
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math 
import time 
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from gym import spaces
from pid_controller import PID
import gym 
from test_model import evaluate
from utils import fig2data
import io
import os
import cv2
from scipy.integrate import solve_ivp
import argparse
import stable_baselines3
import torch
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecCheckNan, VecNormalize
from callbacks import EvalCallback, SaveBestModelCallback
import threading


uinit = [10.0] * 6
umin = -20.0
umax = 20.0
yinit = [0.0] * 6 
delt = 0.001 
slew_rate = 5.0 
disturbance = False
deterministic = True
disturbance_value = 0.0

min_gains = np.array([[0.0]*6, [0.0]*6, [0.0]*6])
max_gains = np.array([[40.0]*6, [2.0]*6, [2.0]*6])

# PID Controller Optimization model's structure follows the alogorithm and template code provided in
#"Reinforcement learning based adaptive PID controller design for control of linear/nonlinear unstable processes" paper 

# CLASS: OptimizerNode
# sets up the dynamics of the input/output to be given to the RL model
#ROS RELATED
# 1. communicates with gazebo by recieving join states topic (current_topic_positions) 
# 2. communicates with the walker node by subscribing to its command topic (command positions) 
# 3. publishes torque commands to the walker node which are then send to gazebo based on walker node's counter frequency (real time)
#OPTIMIZATION RELATED
# 1. state: command_position, current_position, previous_position 
# 2: input: Kp, Kd, Ki 
# 3. output of the model is the command torque which is published to walker node at each step.
# 4.communicates with gym environment via its its state and output
  
class OptimizerNode(Node):
    
    def __init__(
        self,
        uinit=uinit,
        yinit=yinit,
        delt=delt,
        ttfinal=None,
        disturbance=disturbance,
        deterministic=deterministic,
        disturbance_value=disturbance_value,
    ):
        super().__init__('optimizer_node')
        
        
        # ROS2 RELATED ************************************************
        # variables 
        self.current_topic_position = np.zeros(6)
        self.command_position = np.zeros(6)
        self.current_position = np.zeros(6)
        self.previous_position = np.zeros(6)
        self.command_torque = np.zeros(6)
        # Topics
        self.command_position_subscriber_ = self.create_subscription(Float64MultiArray,'/command_position',self.command_position_callback,10)
        self.subJoints_subscriber_ = self.create_subscription(JointState, '/joint_states', self.current_position_callback, 10)
        self.command_torque_publisher = self.create_publisher(Float64MultiArray,'/command_torque',10)
        self.get_logger().info("**************optimizer_node initialized****************")  
        
        # Simulation settings **********************************************************
        self.delt = delt  
        self.ttfinal = ttfinal  
        self.slew_rate = slew_rate
        self.umin = umin
        self.umax = umax
        self.input_low = self.umin
        self.input_high = self.umax
        self.disturbance_value = disturbance_value
        self.uinit = np.array(uinit)
        self.yinit = self.previous_position.copy()
        self.disturbance = disturbance
        self.deterministic = deterministic
        self.xinit = np.array([self.command_position.copy(), self.current_position.copy(), self.previous_position.copy()])
        self.reset()    


    @property
    def state_names(self):
        names = ["Setpoint(k)", "Output(k)", "Output(k-1)"]
        assert len(names) == self.n_states
        return names

    @property
    def input_names(self):
        return ["Kp(k)", "Ki(k)", "Kd(k)"]
    
    @property
    def n_states(self):
        return len(self.get_state())

    @property
    def n_actions(self):
        return self.input_low.shape[0]

    def reset(self):
        
        # create batch of 200 at each iteration 
        #initializes command position vector 
        self.r = np.zeros((200, 6))
        self.r[0] = self.command_position.copy()
        
        sim_time = 200 * self.delt
        self.ttfinal = (
            self.ttfinal
            if self.ttfinal is not None and self.ttfinal < sim_time
            else sim_time
        )
        self.tt = np.arange(0, self.ttfinal, self.delt)  
        self.kfinal = 199 #number of time intervals 
        return self.reset_input()

    def reset_input(self, *args, **kwargs):
        auto = False
        self.Gc = PID(
            [10.0]*6,
            [0.0]*6,
            [0.0]*6,
            setpoint=self.yinit,
            sample_time=self.delt,
            output_limits=(self.umin, self.umax),
            auto_mode=auto,
        )
        self.Gc.set_auto_mode(not auto, last_output=self.uinit)
        self.slew_rate = None
        self.input_low = np.array(min_gains)
        self.input_high = np.array(max_gains)
        
        # Input vector
        self.input = np.zeros((200, 3, 6))
        self.input[0] = np.ones((3, 6)) * np.array([[10.0]*6, [0.0]*6, [0.0]*6])
        self.gains = []
        self.gain_components = []
        return self.reset_env(*args, **kwargs)


    def reset_env(self):
        
        self.k = 0

        # torque vector
        self.u = np.zeros((200, 6))
        self.u[0] = self.uinit.copy()
        self.du = np.zeros((199, 6))

        # environment State vector
        self.x = np.zeros((200, 3, 6))
        self.x[0] = np.array(self.xinit)

        # position vector 
        self.y = np.zeros((200, 6))
        self.y[0] = self.yinit.copy()

        # error vector
        self.E = np.zeros((199, 6))
        return self.r[self.k], self.y[self.k]
    
    def step_env(self, u):
        
        # error: command_position vs. current_position 
        self.E[self.k] = self.r[self.k] - self.y[self.k]
        
        # gazebo torque input, no clipping, no slew rate 
        #if self.slew_rate: u = np.clip(u,self.u[self.k - 1] - self.slew_rate,self.u[self.k - 1] + self.slew_rate)
        #self.u[self.k] = np.clip(self.u[self.k], self.umin, self.umax)
        self.u[self.k] = u        
        
        # updates 
        self.x[self.k + 1] = np.array([self.command_position.copy(), self.current_position.copy(), self.previous_position.copy()])
        self.y[self.k + 1] = self.current_position.copy()
        self.r[self.k + 1]= self.command_position.copy()
        self.previous_position = self.current_position.copy()
        self.current_position = self.current_topic_position.copy()

        self.k = self.k + 1
        return self.r[self.k], self.y[self.k]

    def step(self, Kp, taui, taud):
        Ki = Kp / (taui + 0.01)
        Kd = Kp * taud
        self.Gc.setpoint = self.r[self.k]
        self.Gc.tunings = (Kp, Ki, Kd)
        u = self.Gc(self.y[self.k], self.delt)
        
        self.command_torque = u
        self.input[self.k] = np.array([Kp, Ki, Kd])
        self.gains.append([Kp, taui, taud])
        self.gain_components.append(self.Gc.components)
        
        torque = Float64MultiArray()
        torque.data = self.publish_torque(self.command_torque.copy())
        self.command_torque_publisher.publish(torque)
    
        return self.step_env(u)     
    
    def get_state(self):
        return np.array([self.r[self.k], self.y[self.k], self.y[self.k - 1]])
    
    def ise(self):
        return float(np.sum((self.r[:self.k] - self.y[:self.k]) ** 2))

    def iae(self):
        return float(np.sum(np.abs(self.r[:self.k] - self.y[:self.k])))    

    def get_axis(self, use_sample_instant=True):
        axis = self.tt[: self.k].copy()
        axis_name = "Time (min)"
        if use_sample_instant:
            axis = np.arange(self.k)
            axis_name = "Sampling Instants"
        return axis, axis_name

    def plot(self, save=False, use_sample_instant=True):
        axis, axis_name = self.get_axis(use_sample_instant)
        plt.figure(figsize=(16, 20))
        plt.subplot(3, 1, 1)
        plt.step(
            axis, self.r[: self.k, 0], linestyle="dashed", label="Setpoint", where="post"
        )
        plt.plot(axis, self.y[: self.k, 0], label="Plant Output")
        plt.ylabel("")
        plt.xlabel(axis_name)
        ise = f"{self.ise():.3e}"
        title = f"ISE: {ise}"
        plt.title(title)
        plt.xlim(axis[0], axis[-1])
        plt.grid()
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.step(axis, self.u[: self.k, 0], label="Control Input", where="post")
        plt.ylabel("")
        plt.xlabel(axis_name)
        plt.title("Control Action")
        plt.xlim(axis[0], axis[-1])
        plt.grid()
        plt.legend()

        plt.subplot(3, 1, 3)
        for i in range(1):
            plt.plot(
                axis[:],
                self.input[ : self.k, i, 0],
                label=self.input_names[i],
            )
        plt.ylabel("Value")
        plt.xlabel(axis_name)
        plt.title("Inputs")
        plt.xlim(axis[0], axis[-1])
        plt.grid()
        plt.legend()
        if save:
            plt.tight_layout()
            img = fig2data(plt.gcf())
            plt.close()
            return img

    def plot_gains(self, save=False, use_sample_instant=True):
        axis, axis_name = self.get_axis(use_sample_instant)
        plt.figure(figsize=(16, 12))
        labels = ["$K_p$", "tau_I", "tau_D"]
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(
                axis[ : len(self.gains)],
                np.array(self.gains)[:-1, i, 0],
                label=labels[i],
            )
            plt.ylabel("Value")
            plt.xlabel(axis_name)
            plt.xlim(axis[0], axis[-1])
            plt.grid()
            plt.legend()
        if save:
            plt.tight_layout()
            img = fig2data(plt.gcf())
            plt.close()
            return img

    def plot_actual_gains(self, save=False, use_sample_instant=True):
        axis, axis_name = self.get_axis(use_sample_instant)
        plt.figure(figsize=(16, 12))
        labels = ["$K_p$", "$K_I$", "$K_D$"]
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(
                axis,
                np.array(self.input)[ : self.k, i, 0],
                label=labels[i],
            )
            plt.ylabel("Value")
            plt.xlabel(axis_name)
            plt.xlim(axis[0], axis[-1])
            plt.grid()
            plt.legend()
        if save:
            plt.tight_layout()
            img = fig2data(plt.gcf())
            plt.close()
            return img

    def plot_gain_components(self, use_sample_instant=True):
        axis, axis_name = self.get_axis(use_sample_instant)
        plt.figure(figsize=(16, 9))
        labels = ["Proportional", "Integral", "Derivative"]
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(
                axis[: len(self.gain_components)],
                np.array(self.gain_components)[:-1, i, 0],
                label=labels[i],
            )
            plt.ylabel("Value")
            plt.xlabel(axis_name)
            plt.xlim(axis[0], axis[-1])
            plt.grid()
            plt.legend()
 
    def command_position_callback(self, msg):
        self.command_position = np.array(msg.data)
        
    def current_position_callback(self, msg):
        self.current_topic_position = constrain_angle(np.array([*msg.position]))
        self.current_topic_position = self.current_topic_position[[4, 2, 0, 5, 3, 1]]

    def publish_torque(self, u):
        applied_torque = np.array(u)
        return list(u[[2, 5, 1, 4, 0, 3]])
 
 
# CLASS: GymSystem 
# communicates with the OptimizerNode and gets state and sends actions to it accordingly 
class GymSystem(gym.Env):
    def __init__(
        self,
        uinit=uinit,
        yinit=yinit,
        system=OptimizerNode,
        disturbance=disturbance,
        deterministic=deterministic,
        disturbance_value=disturbance_value,
    ):
        super().__init__()

        self.uinit = uinit
        self.yinit = yinit
        self.disturbance = disturbance
        self.deterministic = deterministic
        self.disturbance_value = disturbance_value
        self.system = system(
            uinit=self.uinit,
            yinit=self.yinit,
            disturbance=self.disturbance,
            deterministic=self.deterministic,
            disturbance_value=self.disturbance_value,
        )

        self.n_actions = (3, 6)
        self.action_space = spaces.Box(-1.0, 1.0, (3,6))
        self.n_states = (3, 6)
        self.observation_space = spaces.Box(
            low=-100.0, high=100.0, shape=self.n_states, dtype=np.float32
        )
       
    def convert_state(self):
        obs = self.system.get_state()
        obs = np.array(obs).astype(np.float32)
        return obs
    
    def convert_action(self, action):
        actions = (action + 1) * (
            self.system.input_high - self.system.input_low
        ) * 0.5 + self.system.input_low
        actions = np.clip(actions, self.system.input_low, self.system.input_high)
        return actions

    def unconvert_action(self, action):
        actions = (2.0 * action - (self.system.input_high + self.system.input_low)) / (
            self.system.input_high - self.system.input_low
        )
        actions = np.clip(actions, -1.0, 1.0)
        return actions
    
    def reset(self):
        _ = self.system.reset()
        obs = self.convert_state()
        return obs

    def get_reward(self, obs):
        # Calculate error and reward
        e = obs[0] - obs[1]
        ##############################ADDEEED################
        sum_abs_e = np.sum(np.abs(e))  # Sum of absolute errors
    
        scale = 0.01 * 6
        e_squared = scale * np.abs(e) ** 2
        e_squared = np.minimum(e_squared, 5.0 *6 )
        ######################### Sum the squared errors
        sum_e_squared = np.sum(e_squared)
        
        tol = (2.0 * 6 - sum_abs_e) if sum_abs_e <= 0.01 * 6 else 0.0
        reward = -sum_e_squared + tol
        return reward
    
    def step(self, action, debug=False):
        # sat_act = np.sum(action[action > 0.96]) + np.sum(action[action < -0.96])
        if debug:
            print("Original: ", action)
        action = self.convert_action(action)
        if debug:
            print("Converted: ", action)
        obs = self.system.step(*action)
        reward = self.get_reward(obs)
        done = bool(self.system.k == self.system.kfinal - 1)
        info = {}
        obs = self.convert_state()
        return obs, reward, done, info
    
    def render(self, mode="human"):
        if mode == "human":
            print("ISE: ", self.system.ise())
            self.system.plot()
        elif mode == "rgb_array":
            return self.system.plot(save=True)
    
    def close(self):
        pass

# Class: Action Repeat 
# (not used currently) mechanism used in the mentioned paper for better optimization (details discussed in paper)
class ActionRepeat(gym.Wrapper):
    def __init__(self, env, amount=1):
        super().__init__(env)
        self.amount = amount

    def step(self, action):
        done = False
        total_reward = 0
        current_step = 0
        while current_step < self.amount and not done:
            obs, reward, done, info = self.env.step(action)
            total_reward += reward
            current_step += 1
        return obs, total_reward, done, info

# Class: EarlyStopping 
# (not used currently) mechanism used in the mentioned paper for better stability/optimization (details discussed in paper)
class EarlyStopping(gym.Wrapper):
    def __init__(self, env, y_lim=[-100, 100]):
        super().__init__(env)
        self.y_lim = y_lim
    #####################################################
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        
        # Check if any value in self.env.system.y[self.env.system.k] is outside the limits
        y_values = self.env.system.y[self.env.system.k]
        if np.any(y_values > self.y_lim[1]) or np.any(y_values < self.y_lim[0]):
            done = True
            reward += -20.0  # Deduct a penalty reward

        return obs, reward, done, info

# Class: Config 
# includes details of the training configuration 
class Config:
    model = "OptimizerNode"
    algo = "PPO"
    logdir = "logs"
    action_repeat = 2
    vec_normalize = True
    early_stopping = False
    mode = "train"

# function for training the RL model 
def run_rl_training():
    env_model = Config.model
    algo = Config.algo
    log_dir = Config.logdir
    action_repeat = False 
    action_repeat_value = Config.action_repeat
    vec_normalize = Config.vec_normalize
    early_stopping = Config.early_stopping
    mode = Config.mode

    env_class = {"OptimizerNode": OptimizerNode,}[env_model]
    print(env_class)

    torch.autograd.set_detect_anomaly(True)
    print("CUDA Available: ", torch.cuda.is_available())
    
    print("Using Early Stopping: ", early_stopping)
    print("Using Action Repeat: ", False, action_repeat_value)
    print("Using gSDE: ", False)
    print("Using VecNormalize: ", vec_normalize)
    print("Algorithm: ", algo)
    extra = "BetterES_SystemFix"
    tag_name = f"CS1_{env_model}_{algo}_AR_{action_repeat}_use_sde_False_ES_{early_stopping}_extra_{extra}"
    print("Run Name: ", tag_name)

    base_log = log_dir
    log_dir = os.path.join(base_log, "CS1", tag_name)

    save_callback = SaveBestModelCallback(check_freq=20000, log_dir=log_dir, verbose=1)

    eval_env = GymSystem(system=env_class)
    if early_stopping:
        eval_env = EarlyStopping(eval_env)
    if action_repeat:
        eval_env = ActionRepeat(eval_env, action_repeat)
    save_image_callback = EvalCallback(
        eval_env=eval_env, eval_freq=50000, log_dir=None, name="Deterministic"
    )
    
    eval_env2 = GymSystem(system=env_class, deterministic=True)
    if early_stopping:
        eval_env2 = EarlyStopping(eval_env2)
    if action_repeat:
        eval_env2 = ActionRepeat(eval_env2, action_repeat_value)
    save_image_callback2 = EvalCallback(
        eval_env=eval_env2, eval_freq=50000, log_dir=log_dir, name="Deterministic"
    )

    callback = CallbackList([save_callback, save_image_callback, save_image_callback2])
    print(callback.callbacks)

    env = GymSystem(system=env_class)
    if early_stopping:
        env = EarlyStopping(env)
    if action_repeat:
        env = ActionRepeat(env, action_repeat)
    env = make_vec_env(lambda: env, n_envs=1, monitor_dir=log_dir)
    if vec_normalize:
        if os.path.exists(os.path.join(log_dir, "vec_normalize.pkl")):
            print("Found VecNormalize Stats. Using stats")
            env = VecNormalize.load(os.path.join(log_dir, "vec_normalize.pkl"), env)
        else:
            print("No previous stats found. Using new VecNormalize instance.")
            env = VecNormalize(env)
    else:
        env.normalize_obs = lambda x: x

    env = VecCheckNan(env, raise_exception=True)

    algo_class = getattr(stable_baselines3, algo)
    model = algo_class("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)

    best_model_path = os.path.join(log_dir, "best_model.zip")
    if os.path.exists(best_model_path) or mode == "test":
        assert os.path.exists(best_model_path), f"Path doesn't exist: {best_model_path}"
        print(f"Found previous checkpoint. Loading from checkpoint. {best_model_path}")
        model = algo_class.load(best_model_path, env)
    print(model)

    if mode == "train":
        tsteps = 500_000
        model.learn(tsteps, reset_num_timesteps=False, callback=callback)

    save_path = Path(log_dir).parts[-2:]
    save_path = os.path.join(*save_path)
    test_log_dir = os.path.join("..", "results", save_path, "test_files", "servo")
    os.makedirs(test_log_dir, exist_ok=True)

    test_env = GymSystem(system=env_class, disturbance=False, deterministic=True)
    if action_repeat:
        test_env = ActionRepeat(test_env, action_repeat)
    evaluate(model, test_env, action_repeat, test_log_dir)

# main function: runs the Optimizernode and run_rl_training concurrently using threading
def main (args = None):
    rclpy.init(args = args)
    node = OptimizerNode()
    
    rl_thread = threading.Thread(target=run_rl_training)
    rl_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    rl_thread.join()

if __name__ == '__main__':
    main()

