import os

from controller import Supervisor

import numpy as np
import gym
from stable_baselines3 import HerReplayBuffer, DDPG, DQN, SAC, PPO
from stable_baselines3.her.goal_selection_strategy import GoalSelectionStrategy
from stable_baselines3.common.noise import NormalActionNoise


class ReachEnvironment(Supervisor, gym.GoalEnv):
    def __init_devices(self):
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        link_names = [
            'base_link',
            'shoulder_link',
            'upper_arm_link',
            'forearm_link',
            'wrist_1_link',
            'wrist_2_link',
            'wrist_3_link'
        ]

        self.motors = [self.getDevice(name) for name in joint_names]
        self.sensors = [self.getDevice(name + '_sensor') for name in joint_names]
        self.touch_sensors = [self.getDevice(name) for name in link_names]
        
        # self.batterySensorEnable(self.__time_step)
        
        # you might want to get devices and enable them
        for sensor in self.sensors + self.touch_sensors:
            sensor.enable(self.__time_step)

    def __init__(self):
        super().__init__()
        
        self.__time_step = 32

        self.motors = None
        self.sensors = None
        self.touch_sensors = None

        self.state = None
        # desired_goal
        self.goal = None
        self.idx = 0
        
        self.action_space = gym.spaces.Box(-1, 1, shape=(6,), dtype=np.float32)
        
        self.observation_space = gym.spaces.dict.Dict({
            'observation': gym.spaces.Box(-1, 1, shape=(12,), dtype=np.float32),
            'achieved_goal': gym.spaces.Box(-1, 1, shape=(12,), dtype=np.float32),
            'desired_goal': gym.spaces.Box(-1, 1, shape=(12,), dtype=np.float32)})
            
        self.reset()

    def reset(self):
        self.idx = 0
        super().reset()

        # Reset the simulation
        self.simulationResetPhysics()
        self.simulationReset()

        # you need to initialize devices here too
        self.__init_devices()
        super().step(self.__time_step)

        self.state = np.zeros(12)
        self.goal = np.asarray([0.25, -0.25, 0.25, 0.25, 0.25, 0.25] + [.0] * 6)
        
        j_pos = np.zeros(6)

        # if np.random.rand() < 1.0:
            # j_pos += np.asarray([1., -1., 1., 1., 1., 1.]) * 0.25 + np.random.randn(6) * 0.05

        for pos, motor in zip(j_pos, self.motors):
            motor.setPosition(pos * np.pi)
        
        # wait until the robot moves to the initial pose
        while super().step(self.__time_step) != -1:
            cur_pos = np.asarray([sensor.getValue() for sensor in self.sensors])
            if all(np.abs(cur_pos - j_pos * np.pi) < 0.001):
                break

        self.state[:6] = j_pos / np.pi
        
        # you might need the inverse kinematics here!
        # pose, velocity -> R^12
        return {'observation': self.state,
                'achieved_goal': self.state,
                'desired_goal': self.goal}

    def step(self, action):
        """
        custom gym environment step
        
        """
        self.idx += 1
        # print(self.idx, action)
        
        # apply action here!
        # change motor torques
        cur_pose = [sensor.getValue() / np.pi for sensor in self.sensors]
        cur_pose = np.asarray(cur_pose)
        
        for motor, torque in zip(self.motors, cur_pose * 0.98 + action * 0.02):
            # motor.setTorque(torque * motor.getMaxTorque())
            motor.setPosition(torque * np.pi)

        # due to MRO, this calls Supervisor.step!
        super().step(self.__time_step)

        # interpret the environment
        # reading sensor values
        next_state = np.zeros(12)

        for i, sensor in enumerate(self.sensors):
            next_state[i] = sensor.getValue() / np.pi
        next_state[6:] = (next_state[:6] - self.state[:6]) * 1000 / self.__time_step / np.pi

        # action is a sequence of torques
        energy_consumption = np.dot(next_state[:6] - self.state[:6], action)

        # check episode terminating conditions
        self.state[:] = next_state

        # goal is changed at the start of episode
        done = False
        collided = False
        out_bound = False
        # print('diff:', np.max(np.abs(self.state[:6] - self.goal[:6])))
        achieved = np.max(np.abs(self.state[:6] - self.goal[:6])) < 5e-2
        
        if achieved:
            reward = 1000
            done = True
            print('achieved!, reward:', reward)
        else:
            reward = -1
    
            limit = np.asarray([1.1] * 6 + [5.0] * 6)
            out_bound = any(np.abs(self.state) > limit)
            collided = [sensor.getValue() > 0.5 for sensor in self.touch_sensors]
            
            if out_bound or any(collided):
                print(out_bound, collided)
                done = True
                reward = -1000
            elif self.idx > 200:
                done = True
    
        # since the goal is all about the robot state,
        # observation is exactly the same with achieved_goal
        return {'observation': np.asarray(self.state),
                'achieved_goal': np.asarray(self.state),
                'desired_goal': np.asarray(self.goal)}, reward, done, {'crashed': out_bound or collided}

    def render(self, mode='human'):
        """
        Do you want to render something else?
        """
        pass

    def compute_reward(self, achieved_goal, desired_goal, info):
        collided = ['collided' in x for x in info]
        achieved = np.max(np.abs(achieved_goal[:, :6] - desired_goal[:, :6]), axis=1) < 5e-2
        
        reward = np.where(achieved, 1000, np.where(collided, -1000, -1))
        return reward


def main():
    env = ReachEnvironment()
    max_episode_length = 100
    gym.envs.register
    online_sampling = True
    goal_selection_strategy = 'future'
    
    model_cls = SAC

    # if os.path.exists('./her_reach_env.zip'):
    #     model = model_cls.load('./her_reach_env', env=env)
    # else:
    # action_noise = NormalActionNoise(mean=np.zeros(6), sigma=0.1 * np.ones(6))
    
    model = model_cls('MultiInputPolicy',
                      env,
                      gamma=0.99,
                      # action_noise=action_noise,
                      replay_buffer_class=HerReplayBuffer,
                      replay_buffer_kwargs=dict(
                          n_sampled_goal=4,
                          goal_selection_strategy=goal_selection_strategy,
                          online_sampling=online_sampling,
                          max_episode_length=max_episode_length
                      ),
                      verbose=1)

    model.learn(1000000)
    model.save('./her_reach_env')


if __name__ == '__main__':
    main()
