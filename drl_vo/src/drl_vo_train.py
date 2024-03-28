#!/usr/bin/env python
#
# revision history: xzt
#  20210604 (TE): first version
#
# usage:
#
# This script is to train the DRL-VO policy using the PPO algorithm.
#------------------------------------------------------------------------------
import csv
import numpy as np
import gym
import turtlebot_gym
import rospy
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
from custom_cnn_full import *


class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq: (int)
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """
    def __init__(self, check_freq: int, log_dir: str, verbose=1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, 'best_model')
        self.best_mean_reward = -np.inf
        # self.metrics_file = open('metrics.csv', mode='w', newline='')
        # self.metrics_writer = csv.writer(self.metrics_file)
        # self.metrics_writer.writerow(['Episode', 'Reward', 'Value', 'Policy_Gradient'])

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
          os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:

          # Retrieve training reward
          x, y = ts2xy(load_results(self.log_dir), 'timesteps')
          if len(x) > 0:
              # Mean training reward over the last 100 episodes
              mean_reward = np.mean(y[-100:])
              if self.verbose > 0:
                print("Num timesteps: {}".format(self.num_timesteps))
                print("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(self.best_mean_reward, mean_reward))

              # New best model, you could save the agent here
              if mean_reward > self.best_mean_reward:
                  self.best_mean_reward = mean_reward
                  # Example for saving best model
                  if self.verbose > 0:
                    print("Saving new best model to {}".format(self.save_path))
                  self.model.save(self.save_path)
                  
        # save model every 100000 timesteps:
        if self.n_calls % (20000) == 0:
          # Retrieve training reward
          path = self.save_path + '_model' + str(self.n_calls)
          self.model.save(path)
	  
        return True
    
    # def _on_rollout_end(self) -> None:
    #     episode_rewards = [ep_info['r'] for ep_info in self.model.ep_info_buffer]
    #     episode_values = [value[0] for value in self.model.rollout_buffer.values]
        
    #     policy_gradients = []
    #     for obs in self.model.rollout_buffer.observations:
    #         obs_tensor = torch.as_tensor(obs, dtype=torch.float32)
    #         _, action_prob = self.model.policy.forward(obs_tensor)
    #         log_prob = torch.log(action_prob)
    #         grad_log_prob = torch.autograd.grad(log_prob, self.model.policy.parameters(), retain_graph=True)
    #         flat_grad_log_prob = torch.cat([g.contiguous().view(-1) for g in grad_log_prob])
    #         policy_gradients.append(flat_grad_log_prob.detach().numpy())

    #     for i in range(len(episode_rewards)):
    #         self.metrics_writer.writerow([self.num_timesteps, episode_rewards[i], episode_values[i], policy_gradients[i]])

    # def _on_training_end(self) -> None:
    #     self.metrics_file.close()

# create ros node:
rospy.init_node('env_test', anonymous=True, log_level=rospy.WARN) #log_level=rospy.ERROR)   

# Create log dir
log_dir = rospy.get_param('~log_dir', "./runs/")
os.makedirs(log_dir, exist_ok=True)

# Create and wrap the environment
env = gym.make('drl-nav-v0')
env = Monitor(env, log_dir) #, allow_early_resets=True)  # in order to get rollout log data
#env = DummyVecEnv([lambda: env])
obs = env.reset()

# policy parameters:
policy_kwargs = dict(
    features_extractor_class=CustomCNN,
    features_extractor_kwargs=dict(features_dim=256),
    net_arch=[dict(pi=[256], vf=[128])]
)

# raw training:
#model = PPO("CnnPolicy", env, policy_kwargs=policy_kwargs, learning_rate=1e-3, verbose=2, tensorboard_log=log_dir, n_steps=512, n_epochs=10, batch_size=128) #, gamma=0.96, ent_coef=0.1, vf_coef=0.4) 

# continue training:
kwargs = {'tensorboard_log':log_dir, 'verbose':2, 'n_epochs':10, 'n_steps':512, 'batch_size':128,'learning_rate':5e-5}
model_file = rospy.get_param('~model_file', )#"/home/lab423/drl_GMM_Hallway_no_collision_runs/best_model.zip")
model = PPO.load(model_file, env=env, **kwargs)

# Create the callback: check every 1000 steps
callback = SaveOnBestTrainingRewardCallback(check_freq=5000, log_dir=log_dir)
model.learn(total_timesteps=2000000, log_interval=5, tb_log_name='drl_vo_policy', callback=callback, reset_num_timesteps=True)

# Saving final model
model.save("drl_vo_model")
print("Training finished.")
env.close()
