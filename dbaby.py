import gym
import numpy as np
from itertools import count
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.autograd as autograd
from torch.autograd import Variable
from torch.utils.data import TensorDataset, DataLoader
import npickle
import types
import matplotlib.pyplot as plt
%matplotlib inline

gamma, seed, batch = 0.99, 543, 10
D = 105*80

#env = gym.make("PongDeterministic-v3")
env = gym.make("Pong-v3")
#env = gym.make("SpaceInvaders-v3") # environment info
env.seed(seed)
torch.manual_seed(seed)

# Get from apple picking robot environment
valid_actions = range(env.action_space.n) #number of valid actions in the specific game   
print ('action count:', env.action_space.n)

class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()
        self.affine1 = nn.Linear(D, 50)
        nn.init.xavier_normal(self.affine1.weight)
        self.affine2 = nn.Linear(50, len(valid_actions))
        nn.init.xavier_normal(self.affine2.weight)

        self.saved_actions = []
        self.rewards = []

    def forward(self, x):
        x = F.relu(self.affine1(x))
        action_scores = self.affine2(x)
        return F.softmax(action_scores)

policy = Policy()
policy.cuda()
optimizer = optim.RMSprop(policy.parameters(), lr=1e-3)

def prepro(I):
    # 105 x 80
    I = I[::2,::2,2] # downsample by factor of 2, choose colour 2 to improve visibility in other games   
    I[I == 17] = 0 # erase background (background type 1)   
    I[I == 192] = 0 # erase background (background type 2)   
    I[I == 136] = 0 # erase background (background type 3)   
    I[I != 0] = 1 # everything else (paddles, ball) just set to 1 
    # plt.imshow(I) 
    # plt.show()

    # output should be binary image

    return I.astype(np.float).ravel() # 2D array to 1D array (vector)
    

def finish_episode():
    R = 0
    rewards = []
    for r in policy.rewards[::-1]:
        if r != 0: R = 0 # reset the sum, since this was a game boundary (pong specific!)   
        R = r + gamma * R
        rewards.insert(0, R)
    #print (rewards)
    rewards = torch.Tensor(rewards).cuda()
    rewards = (rewards - rewards.mean()) # / (rewards.std() + np.finfo(np.float32).eps)  
    tmp = rewards.std()  
    if tmp > 0.0 : rewards /= tmp #fixed occasional zero-divide   
    for action, r in zip(policy.saved_actions, rewards):
        action.reinforce(r)
    optimizer.zero_grad()
    autograd.backward(policy.saved_actions, [None for _ in policy.saved_actions])
    optimizer.step()
    del policy.rewards[:]
    del policy.saved_actions[:]


running_reward = None 
lgraphx, lgraphy = [], [] #used for plotting graph 
for episode_number in count(1):
    observation = env.reset() # reset bot and apple positions
    prev_x = None # used in computing the difference frame    
    reward_sum = 0  

    for t in range(10000): # Don't infinite loop while learning
        cur_x = prepro(observation)   
        state = cur_x - prev_x if prev_x is not None else np.zeros(D)   
        prev_x = cur_x  

        state = torch.from_numpy(state).float().unsqueeze(0)
        
        probs = policy(Variable(state))

        # With or without GPU
        if GPU == True:
            probs.cuda()
        
        actiong = probs.multinomial()
        policy.saved_actions.append(actiong)
        action = actiong.data

        observation, reward, done, _ = env.step(action[0,0])
        reward_sum += reward 
        policy.rewards.append(reward)

        if done:
            break

    accumulate = min([episode_number, 10])
    running_reward = reward_sum if running_reward is None else running_reward * (1-1/accumulate) + reward_sum * (1/accumulate)
    print ('episode: {}, reward: {}, mean reward: {:3f}'.format(episode_number, reward_sum, running_reward))
    lgraphx.append(episode_number)
    lgraphy.append(running_reward)
    
    if episode_number % batch == 0:
        print('Episode {}\tLast length: {:5d}\tAverage length: {:.2f}'.format(
            episode_number, t, running_reward))
        plt.plot(lgraphx, lgraphy)
        plt.show() 
        finish_episode()

