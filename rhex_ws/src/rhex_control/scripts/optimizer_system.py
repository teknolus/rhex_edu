#!/usr/bin/env python3

# Instructions:
# Default configuration is for training. To test the best stored model, in config class set up mode as "test"
# run the following commands: 
# ros2 launch rhex_gazebo start_sim.launch.py 
# ros2 launch rhex_control walker_node.launch.py
# ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# ros2 launch rhex_control optimizer_launcher.launch.py

# RL model mostly based on the paper: 
# Lee, Daesoo & Lee, Seung & Yim, Solomon. (2020). 
# Reinforcement learning-based adaptive PID controller for DPS. Ocean Engineering. 216. 10.1016/j.
# oceaneng.2020.108053.  
 
import rclpy
from rclpy.clock import Clock
import rclpy.parameter
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from miscellaneous import constrain_angle
import numpy as np
import math 
import time 
import matplotlib.pyplot as plt
import io
import os
import cv2

import gym 
from gym import spaces
from scipy.integrate import solve_ivp
import argparse
import stable_baselines3
import torch
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecCheckNan, VecNormalize

import threading
import roslibpy

# initial torque value 
uinit = np.ones(6) * 5.0
# max, min values for kp and kd 
umin = [[0.0]*6, [0.0]*6]
umax = [[20.0]*6, [0.5]*6 ]
yinit = np.zeros(6)
# optimization step time duration (real time)
delt = 0.001



class OptimizerSystem:
    """
    The OptimizerSystem class is designed to:
    1. Compute the PD controller's torque output. 
    2. Communicate with walker node via subscribing to its current and command positions and velocity topics and publishing torque commands. 
    2: Communicate with gym environment by sending state input (command position, current position, previous position) and recieving action output (Kp, Kd).

    It integrates with ROS through the roslibpy library and allows for the subscription to and publication
    of relevant topics. The class also manages simulation parameters, environment states, and system inputs.
    """
    
    def __init__(
        self,
        uinit=uinit,
        yinit=yinit,
        delt=delt,
        ttfinal=None,
    ):
        self.command_position = np.zeros(6)
        self.command_velocity = np.zeros(6)
        
        self.current_position = np.zeros(6)
        self.current_velocity = np.zeros(6)
        
        self.previous_position = np.zeros(6)
        self.previous_velocity = np.zeros(6)
        
        self.command_torque = np.zeros(6)


        # ROSLIBPY RELATED -- handles subscriptions and publications 
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()


        # ROS subscriptions
        self.command_position_subscriber = roslibpy.Topic(self.client, '/command_position', 'std_msgs/Float64MultiArray')
        self.command_velocity_subscriber = roslibpy.Topic(self.client, '/command_velocity', 'std_msgs/Float64MultiArray')
        self.current_position_subscriber = roslibpy.Topic(self.client, '/current_position', 'std_msgs/Float64MultiArray')
        self.current_velocity_subscriber = roslibpy.Topic(self.client, '/current_velocity', 'std_msgs/Float64MultiArray')
        
        self.command_position_subscriber.subscribe(self.command_position_callback)
        self.command_velocity_subscriber.subscribe(self.command_velocity_callback)
        self.current_position_subscriber.subscribe(self.current_position_callback)
        self.current_velocity_subscriber.subscribe(self.current_velocity_callback)

        # ROS publisher
        self.command_torque_publisher = roslibpy.Topic(self.client, '/command_torque', 'std_msgs/Float64MultiArray')

        # Simulation settings 
        self.delt = delt  # Time step duration (real time)
        self.ttfinal = ttfinal  
        self.input_low = np.array(umin)
        self.input_high = np.array(umax)
        self.uinit = np.array(uinit)
        self.yinit = np.array([self.current_position, self.current_velocity])
        self.xinit = np.array([[self.command_position, self.command_velocity], [self.current_position, self.current_velocity], [self.previous_position, self.previous_velocity]])
        
        self.reset()    
        
    def command_position_callback(self, message):
        self.command_position = np.array(message['data'])

    def command_velocity_callback(self, message):
        self.command_velocity = np.array(message['data'])
        
    def current_position_callback(self, message):
        self.previous_position = self.current_position
        self.current_position = np.array(message['data'])
        
    def current_velocity_callback(self, message):
        self.previous_velocity = self.current_velocity
        self.current_velocity = np.array(message['data'])

    def publish_torque(self, u):
        torque = roslibpy.Message({'data': list(u[[2, 5, 1, 4, 0, 3]])})
        self.command_torque_publisher.publish(torque)
        
    @property
    def state_names(self):
        names = ["Position Setpoint(k)",  "Velocity Setpoint(k)", "Position Output(k)", "Velocity Output (k)", "Velocity Output(k-1)", "Position Output(k-1)"]
        assert len(names) == 6 
        return names

    @property
    def input_names(self):
        return ["Kp(k)", "Kd(k)"]
    
    @property
    def n_states(self):
        return (3, 2, 6)

    @property
    def n_actions(self):
        return (2, 6)

    def reset(self):
        # environment resets after 2000 time steps (1999 intervals)  
        
        # stores command_position and command_velocity at each step 
        self.r = np.zeros((2000, 2, 6))
        self.r[0] = np.array([self.command_position, self.command_velocity])
        
        sim_time = 2000 * self.delt
        
        self.ttfinal = (
            self.ttfinal
            if self.ttfinal is not None and self.ttfinal < sim_time
            else sim_time
        )
        
        self.tt = np.arange(0, self.ttfinal, self.delt) 
         
        self.kfinal = 1999
        
        return self.reset_input()

    def reset_input(self, *args, **kwargs):
        
        # stores the output (Kp, Kd) of the optimization system at each step 
        self.input = np.zeros((2000, 2, 6))
        self.input[0] = np.ones((2, 6)) * np.array([[10.0]*6, [0.3]*6])
        return self.reset_env(*args, **kwargs)

    def reset_env(self):
        
        self.k = 0

        # stores command_torque calculated from kp and kd at each step 
        self.u = np.zeros((2000, 6))
        self.u[0] = self.uinit
        self.du = np.zeros((1999, 6))

        # stores environment state at each step (command pos and vel, current pos and vel, previous pos and vel)
        self.x = np.zeros((2000, 3, 2, 6))
        self.x[0] = np.array(self.xinit)

        # stores position and velocity at each step  
        self.y = np.zeros((2000, 2, 6))
        self.y[0] = self.yinit

        # stores command vs current position and velocity error at each step
        self.E = np.zeros((1999, 2, 6))
        return self.r[self.k], self.y[self.k]
        
    def step(self, Kp, Kd, mode = "train"):
        
        # output of the optimization system 
        self.input[self.k] = np.array([Kp, Kd])
        
        # calculated command torque 
        u = Kp * (self.command_position - self.current_position) + Kd * (self.command_velocity - self.current_velocity)
        
        # max and min command torques 
        min_limit = -10.0
        max_limit = 10.0
        u = np.clip(u, min_limit, max_limit)
        
        self.publish_torque(u)

        self.E[self.k] = self.r[self.k] - self.y[self.k]
        self.u[self.k] = u        
        
        # updates 
        self.x[self.k + 1] = np.array([[self.command_position, self.command_velocity], [self.current_position,  self.current_velocity], self.y[self.k]])
        self.y[self.k + 1] = np.array([self.current_position, self.current_velocity])
        self.r[self.k + 1]= np.array([self.command_position, self.command_velocity])
        

        self.k = self.k + 1
        
        # continuously run if test mode 
        if mode == "test" and self.k >= 1998:
            self.k = 0
            
        return self.r[self.k], self.y[self.k]

    def get_state(self):
        return np.array([self.r[self.k], self.y[self.k], self.y[self.k - 1]])
    
    def ise(self):
        return float(np.sum((self.r[:self.k] - self.y[:self.k]) ** 2))

    def iae(self):
        return float(np.sum(np.abs(self.r[:self.k] - self.y[:self.k])))    
    
    def shutdown(self):
        self.command_position_subscriber.unsubscribe()
        self.command_velocity_subscriber.unsubscribe()
        self.current_position_subscriber.unsubscribe()
        self.client.terminate()
 
class GymSystem(gym.Env):
    """
    The GymSystem is designed to communicate with the Optimizersystem and output Kp and Kd at each step based on the given state. 
    """
    def __init__(self,system_instance ):
        super().__init__()
        
        # Use the passed instance of OptimizerSystem
        self.system = system_instance

        # action is Kp, Kd
        self.n_actions = (2, 6)
        self.action_space = spaces.Box(-1.0, 1.0, shape = self.n_actions)
        
        self.n_states = (3, 2, 6)
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
        sum_abs_e = np.sum(np.abs(e))  # Sum of absolute errors
    
        scale = 1e-3
        e_squared = scale * np.abs(e) ** 2
        #e_squared = np.abs(e) ** 2
        #e_squared = np.minimum(e_squared, 5.0)
        sum_e_squared = np.sum(e_squared)
        tol = (2.0 - sum_abs_e) if sum_abs_e <= 0.01  else 0.0
        reward = -sum_e_squared + tol
        
        return reward
    
    def step(self, action, mode = "train", debug=False):
        if debug:
            print("Original: ", action)
        action = self.convert_action(action)
        if debug:
            print("Converted: ", action)
        if mode == "train":
            obs = self.system.step(*action, mode = "train")
        elif mode == "test":
            obs = self.system.step(*action, mode = "test")
        reward = self.get_reward(obs)
        done = bool(self.system.k == self.system.kfinal - 1)
        info = {}
        obs = self.convert_state()
        return obs, reward, done, info
    
    def render(self, mode="human"):
        pass
    
    def close(self):
        pass

class ActionRepeat(gym.Wrapper):
    """ 
    mechanism used in the mentioned paper for better optimization (details discussed in paper)
    """
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
    
class EarlyStopping(gym.Wrapper):
    """ 
    mechanism used in the mentioned paper for better stability/optimization (details discussed in paper)
    """
    def __init__(self, env, y_lim=[-100, 100]):
        super().__init__(env)
        self.y_lim = y_lim
        
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        
        # Check if any value in self.env.system.y[self.env.system.k] is outside the limits
        y_values = self.env.system.y[self.env.system.k]
        if np.any(y_values > self.y_lim[1]) or np.any(y_values < self.y_lim[0]):
            done = True
            reward += -20.0  # Deduct a penalty reward

        return obs, reward, done, info

class Config:
    """
    includes details of the training configuration 
    """
    model = "OptimizerSystem"
    algo = "PPO"
    logdir = "logs"
    action_repeat = 2
    vec_normalize = True
    early_stopping = True
    mode = "train"  # Change to "test" when you want to test the model

def run_rl_training(system):
    # trains the model and stores best model 
    env_model = Config.model
    algo = Config.algo
    log_dir = Config.logdir
    action_repeat = True 
    action_repeat_value = Config.action_repeat
    vec_normalize = Config.vec_normalize
    early_stopping = Config.early_stopping
    mode = Config.mode

    env_class = {"OptimizerSystem": OptimizerSystem,}[env_model]
    print(env_class)

    torch.autograd.set_detect_anomaly(True)
    print("CUDA Available: ", torch.cuda.is_available())
    
    print("Using Early Stopping: ", early_stopping)
    print("Using Action Repeat: ", True , action_repeat_value)
    print("Using gSDE: ", False)
    print("Using VecNormalize: ", vec_normalize)
    print("Algorithm: ", algo)
    extra = "BetterES_SystemFix"
    tag_name = f"Rhex_state_10_Constant_Zero_Command_{env_model}_{algo}_AR_{action_repeat}_use_sde_False_ES_{early_stopping}_extra_{extra}"
    print("Run Name: ", tag_name)

    base_log = log_dir
    log_dir = os.path.join(base_log, "Rhex_state_10" , tag_name)

    # Pass the system instance to GymSystem
    env = GymSystem(system_instance=system)
    
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

    best_reward = -np.inf
    
    if mode == "train":
        tsteps = 500_000
        for step in range (tsteps):
            model.learn(400, reset_num_timesteps = False)
            current_reward = evaluate_model(model, env) 

            if current_reward > best_reward:
                best_reward = current_reward
                model.save(os.path.join(log_dir, "best_model"))
                print(f"New best model saved with reward: {best_reward}")

def evaluate_model(model, env, num_episodes=5):
    total_reward = 0.0
    for _ in range(num_episodes):
        obs = env.reset()
        done = False
        while not done:
            action, _ = model.predict(obs)
            obs, reward, done, _ = env.step(action)
            total_reward += reward
    average_reward = total_reward / num_episodes
    return average_reward

def main(args=None):
    
    opt_sys = OptimizerSystem()  
    
    if Config.mode == "train":
        # Training thread
        rl_thread = threading.Thread(target=run_rl_training, args=(opt_sys,))
        rl_thread.start()
        rl_thread.join()
    
    elif Config.mode == "test":
        model = PPO.load(os.path.join("logs/Rhex_state_10/Rhex_state_10_Constant_Zero_Command_OptimizerSystem_PPO_AR_True_use_sde_False_ES_True_extra_BetterES_SystemFix/", "best_model"))  
        env = GymSystem(system_instance=opt_sys)  # Initialize the environment
        obs = env.reset()
        try:
            while True:
                action, _ = model.predict(obs)
                obs, reward, done, info = env.step(action, mode = "test")
        except KeyboardInterrupt:
            print("Evaluation interrupted by user")
        finally:
            opt_sys.shutdown()

    opt_sys.shutdown()

if __name__ == '__main__':
    main()

