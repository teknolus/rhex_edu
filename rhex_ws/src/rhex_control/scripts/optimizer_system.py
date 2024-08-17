#!/usr/bin/env python3

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


uinit = np.ones(6) * 5.0
umin = [[0.0]*6, [0.0]*6]
umax = [[20.0]*6, [1.0]*6 ]
yinit = np.zeros(6)
delt = 0.002
slew_rate = 5.0 
disturbance = False
deterministic = True
disturbance_value = 0.0


class OptimizerSystem:
    """
    The OptimizerSystem class is designed to:
    1. Compute the PD controller's torque output. 
    2. Communicate with walker node via subscribing to its current and command positions and velocity topics and publishing torque commands. 
    2: Communicate with gym environment by sending state input (command position, current position, previous position) and recieving action output (Kp, Kd).

    It integrates with ROS through the roslibpy library and allows for the subscription to and publication
    of relevant topics. The class also manages simulation parameters, environment states, and system inputs.

    Attributes:
    -----------
    client : roslibpy.Ros
        The ROS client used for communication.
        
    command_position : numpy.ndarray
        Array to store the command position values received from ROS topics.
    command_velocity : numpy.ndarray
        Array to store the command velocity values received from ROS topics.
    current_position : numpy.ndarray
        Array to store the current position values received from ROS topics.
    current_velocity : numpy.ndarray
        Array to store the current velocity values received from ROS topics.
    command_torque : numpy.ndarray
        Array to store the command torque values to be published.
    previous_position : numpy.ndarray
        Array to store the previous position values.
        
    delt : float
        Time step for the simulation.
    ttfinal : float
        Final time for the simulation.
    umin : numpy.ndarray
        Minimum limits for the input vector.
    umax : numpy.ndarray
        Maximum limits for the input vector.
    uinit : numpy.ndarray
        Initial input values.
    yinit : numpy.ndarray
        Initial output values.
    xinit : numpy.ndarray
        Initial state values for the system.
        
    r : numpy.ndarray
        Array to store the reference trajectory.
    u : numpy.ndarray
        Array to store the input torque values.
    du : numpy.ndarray
        Array to store the changes in input torque.
    x : numpy.ndarray
        Array to store the state values of the system.
    y : numpy.ndarray
        Array to store the output values of the system.
    E : numpy.ndarray
        Array to store the error values between the reference and output.
    k : int
        Index for the current simulation step.
    kfinal : int
        Final index for the simulation steps.
    input : numpy.ndarray
        Array to store the input values for the control system.

    Methods:
    --------
    __init__(self, uinit, yinit, delt, ttfinal=None, disturbance, deterministic, disturbance_value):
        Initializes the OptimizerSystem with the given parameters.
        
    command_position_callback(self, message):
        Callback function for updating the command position from ROS topics.
        
    command_velocity_callback(self, message):
        Callback function for updating the command velocity from ROS topics.
        
    current_position_callback(self, message):
        Callback function for updating the current position from ROS topics.
        
    current_velocity_callback(self, message):
        Callback function for updating the current velocity from ROS topics.
        
    publish_torque(self, u):
        Publishes the computed torque values to the corresponding ROS topic.
        
    state_names(self):
        Property to return the names of the states being tracked.
        
    input_names(self):
        Property to return the names of the inputs being controlled.
        
    n_states(self):
        Property to return the number of states in the system.
        
    n_actions(self):
        Property to return the number of actions (or inputs) in the system.
        
    reset(self):
        Resets the system's state, input, and output vectors to their initial values.
        
    reset_input(self, *args, **kwargs):
        Resets the input vectors in preparation for a new simulation episode.
        
    reset_env(self):
        Resets the environment state, position, and error vectors for a new simulation episode.
        
    step(self, Kp, Kd):
        Performs a simulation step by computing the control input based on the given Kp and Kd values,
        updating the system's state, and applying the input to the system.
        
    get_state(self):
        Returns the current state of the system as an array.
        
    ise(self):
        Computes and returns the integral of the squared error (ISE) over the simulation period.
        
    iae(self):
        Computes and returns the integral of the absolute error (IAE) over the simulation period.
        
    shutdown(self):
        Unsubscribes from ROS topics and terminates the ROS client connection.
    """
    
    def __init__(
        self,
        uinit=uinit,
        yinit=yinit,
        delt=delt,
        ttfinal=None,
    ):
        
        # ROSLIBPY RELATED ************************************************
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()

        self.command_position = np.zeros(6)
        self.command_velocity = np.zeros(6)
        
        self.current_position = np.zeros(6)
        self.current_velocity = np.zeros(6)
        
        self.previous_position = np.zeros(6)
        self.previous_velocity = np.zeros(6)
        self.command_torque = np.zeros(6)

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

        # Simulation settings **********************************************************
        self.delt = delt  
        self.ttfinal = ttfinal  
        self.umin = np.array(umin)
        self.umax = np.array(umax)
        self.input_low = self.umin
        self.input_high = self.umax
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
        assert len(names) == 6 #self.n_states
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
        
        
        self.input = np.zeros((2000, 2, 6))
        self.input[0] = np.ones((2, 6)) * np.array([[10.0]*6, [0.3]*6])
        return self.reset_env(*args, **kwargs)

    def reset_env(self):
        
        self.k = 0

        # torque vector
        self.u = np.zeros((2000, 6))
        self.u[0] = self.uinit
        self.du = np.zeros((1999, 6))

        # environment State vector
        self.x = np.zeros((2000, 3, 2, 6))
        self.x[0] = np.array(self.xinit)

        # position vector 
        self.y = np.zeros((2000, 2, 6))
        self.y[0] = self.yinit

        # error vector
        self.E = np.zeros((1999, 2, 6))
        return self.r[self.k], self.y[self.k]
        
    
    def step(self, Kp, Kd, mode = "train"):
        
        self.input[self.k] = np.array([Kp, Kd])
        
        
        u = Kp * (self.command_position - self.current_position) + Kd * (self.command_velocity - self.current_velocity)
        
        min_limit = -20.0
        max_limit = 20.0

        # Apply min-max limits to u
        u = np.clip(u, min_limit, max_limit)
        
        #print(self.command_position)
        #print("self.command_position", self.command_position)
        #print("self.current_position", self.current_position)
        
        self.publish_torque(u)

        # error: command_position vs. current_position 
        self.E[self.k] = self.r[self.k] - self.y[self.k]
        
        # gazebo torque input, no clipping, no slew rate 
        #if self.slew_rate: u = np.clip(u,self.u[self.k - 1] - self.slew_rate,self.u[self.k - 1] + self.slew_rate)
        #self.u[self.k] = np.clip(self.u[self.k], self.umin, self.umax)
        self.u[self.k] = u        
        
        # updates 
        self.x[self.k + 1] = np.array([[self.command_position, self.command_velocity], [self.current_position,  self.current_velocity], [self.previous_position, self.previous_velocity]])
        self.y[self.k + 1] = np.array([self.current_position, self.current_velocity])
        self.r[self.k + 1]= np.array([self.command_position, self.command_velocity])
        

        self.k = self.k + 1
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
 
# CLASS: GymSystem 
# communicates with the OptimizerNode and gets state and sends actions to it accordingly 
class GymSystem(gym.Env):
    def __init__(self,system_instance ):
        super().__init__()

        # Use the passed instance of OptimizerSystem
        self.system = system_instance

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
        ######################### Sum the squared errors
        sum_e_squared = np.sum(e_squared)
        tol = (2.0 - sum_abs_e) if sum_abs_e <= 0.01  else 0.0
        reward = -sum_e_squared + tol
        
        return reward
    
    def step(self, action, mode = "train", debug=False):
        # sat_act = np.sum(action[action > 0.96]) + np.sum(action[action < -0.96])
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
    model = "OptimizerSystem"
    algo = "PPO"
    logdir = "logs"
    action_repeat = 2
    vec_normalize = True
    early_stopping = True
    mode = "train"  # Change to "test" when you want to test the model

# function for training the RL model 
def run_rl_training(system):
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
    opt_sys = OptimizerSystem()  # Ensure OptimizerSystem is using roslibpy internally

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

