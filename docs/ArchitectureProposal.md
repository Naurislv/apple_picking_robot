
# Table of content

1. [Architecture](#architecture)
   - [What it is self driving apple picking robot and what it is not](#what-it-is-self-driving-apple-picking-robot-and-what-it-is-not)
   - [General concept of self driving car](#general-concept-of-self-driving-car)
   - [Apple picking robot architecture](#apple-picking-robot-architecture)
1. [Paper review](#paper-review)
   - [Ground Language Learning in a Simulated 3D World](#ground-language-learning-in-a-simulated-3d-world)
   - [Deep Reinforcement Learning from Human Preferences](#deep-reinforcement-learning-from-human-preferences)
   - [Mechanical Design of a Cartesian Manipulator for Warehouse Pick and Place](#mechanical-design-of-a-cartesian-manipulator-for-warehouse-pick-and-place)
   - [Neural Task Programming Learning to Generalize Across Hierarchical Tasks](#neural-task-programming-learning-to-generalize-across-hierarchical-tasks)
   - [Rainbow: Combining Improvements in Deep Reinforcement Learning](#rainbow-combining-improvements-in-deep-reinforcement-learning)
   - [Learning to Push by Grasping: Using multiple tasks for effective learning](#learning-to-push-by-grasping-using-multiple-tasks-for-effective-learning)

# Architecture

## What it is self driving apple picking robot and what it is not

Before to start thiking of what it is we should understand __what is the minimum we want to build__, we need to create description of [Minimum Viable Product](https://en.wikipedia.org/wiki/Minimum_viable_product) although we are not building product we need to understand something about what we are building.

Apple picking Robot characteristics of minimum functionality point of view dividing by modules:

|    Functionality             |        Module            |    Inputs            |       Possible How       | Level   |
|------------------------------|--------------------------|----------------------|--------------------------|---------|
| Locate apples                | Perception, Detection    | Camera, Depth camera | Deep Learning            | easy    |
| Navigate to closest apple    | Route Planning           | Camera, Map, Other   | Planning algorithms      | medium  |
| Scene understanding          | Perception, Detection    | Camera, Other        | Localization algos, DL   | medium  |
| Pick apple and put in basket | Perception, Planning     | Camera, Other        | Motion planning, DL      | hard    |

Note that robot must be controlable which means that we cannot use some black box which do everything and hope for the best.

Now that we know what robot have to do let's understand what it is. It is simplified self driving car which have starting point and destination point. It is robot, with robotic arm. And those components are not related altough both system may use same sensory inputs which means that both components must be interconnected.

## Simplified self driving car architecture

General architecture proposed for self driving cars. Briefly describes how we would build self driving car architecture and later discuss how we can apply this to our architecture.

![images](./images/system_arch.png)

### Sensor subsystem

Collect and pass camera, Depth camera, Lidar, Radar, GPS data to Perception subsystem.

### Perception subsystem

Responsible for detecting apples, obstacles, boundaries as well as localize itself in environment.

### Planning subsystem

1. Route planning

    The route planning component is responsible for high-level decisions about the path of the vehicle between two points on a map; for example which roads, highways, or freeways to take. This component is similar to the route planning feature found on many smartphones or modern car navigation systems.

2. Prediction

    The prediction component estimates what actions other objects might take in the future. For example, if another vehicle were identified, the prediction component would estimate its future trajectory.

3.  Behavioral planning

    The behavioral planning component determines what behavior the vehicle should exhibit at any point in time. For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component.

4. Trajectory planning

    Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.

### Controll subsystem

Make sure that vehicle follow the path issued by Planning subsystem. This subsystem is responsible for sending following values to robot:

1. Acceleration
2. Braking
3. Steering

## Apple picking robot architecture

![apple_picking_robot](images/apple_picking_robot.png)

# Paper review

* [Ground Language Learning in a Simulated 3D World](https://arxiv.org/abs/1706.06551), Reviewed
* [Deep Reinforcement Learning from Human Preferences](https://arxiv.org/pdf/1706.03741.pdf), Reviewed
* [Mechanical Design of a Cartesian Manipulator for Warehouse Pick and Place](http://juxi.net/papers/ACRV-TR-2017-02.pdf), Reviewed
* [Neural Task Programming: Learning to Generalize Across Hierarchical Tasks](https://arxiv.org/abs/1710.01813?utm_campaign=Revue%20newsletter&utm_medium=Newsletter&utm_source=The%20Wild%20Week%20in%20AI)
* [Rainbow: Combining Improvements in Deep Reinforcement Learning](https://arxiv.org/abs/1710.02298?utm_campaign=Revue%20newsletter&utm_medium=Newsletter&utm_source=The%20Wild%20Week%20in%20AI)
* [Learning to Push by Grasping: Using multiple tasks for effective learning](https://arxiv.org/pdf/1609.09025.pdf)

## [Ground Language Learning in a Simulated 3D World](https://arxiv.org/abs/1706.06551)

### Abstract

Novel paradigm for simulating language learning and understanding. The goal of the paper is to teach the agent to do specific tasks using natural language command and visual input, for example in this implementation.

The agent will receive positive points for picking specified object, and negative points for other objects. Optimization step is performed when the replay memory buffer is full, and then cleared.

Address longstanding challenge of relating language to the physical world. Talking to machines in a natural language and tell them what to do.

### Up-sides

* The speed of learning new semantics increases as more semantics are learned.
* No rule based approach. System learns how to act based on input phrase.
* End-to-End system. Based on pixel inputs.
* Generelizes well. Authors contructed more complext tasks pretraining to other characteristics of human language understanding such as understanding new phrases and able to find not seen before object.
* Unofficial. Resulted code can be found, (PyTorch, Python 2.7), enviroment code in LUA, no comparable results with DeepMinds work: https://github.com/dai-dao/Grounded-Language-Learning-in-Pytorch

### Useful for us

* How to process and relate textual input to surrounding environment - 3D world.
* If everything works out great then we could build more general system for picking different objects and not only.

### Challenges

* This approach have been tested only in Virtual Enviroment so there are no results and challenges described for real life scenario. Learning is very slow which mean that in real life we should have to figure out how to maintain __very long and repeatable (by machine) learning mechanics or we could train in supervised manner before.__
* Authors experimented in a complex virtual 3D world however actions was limited to move to the place we need and there wasn't picking up and moving to somewhere actions involved which raises a question - __how we would solve this and if this is right approach for this job?__ Agents can provide multiple simultaneous actions to control movement (forward/back, strafe left/right, crouch, jump), looking (up/down, left/right) and tagging (in laser tag levels with opponent bots).
* Reward system costruction in real life is much harder than in virtual environment, this could be challenge as we may need to create separate system just to reward our agen (apple picker).
* Altough 3D simulated world was randomly filled with millions of different objects with different colors and different places. Enviroment is very simplified uniformly coloring objects and surrounding as well as image inputs always are clear, steady and without any noise which comes in real life. Figure 1 in paper: "This  exploration  and  selection  behaviour  emerges  entirely  from  the reward-driven learning and is not preprogrammed.  When training on a task such as this."
there  are  billions  of  possible  episodes  that  the  agent  can  experience,  containing  different objects in different positions across different room layouts."
* RL policy optimized by Advantage Actor Critic algorithm. They run 32 async thread parallel (32 agents) which itself is a big challenge for physical world. 6 million episodes learning from scratch.
* Comment from [Reddit](https://www.reddit.com/r/MachineLearning/comments/6im2r8/r_from_deepmind_grounded_language_learning_in_a/): "Their solutions don't scale to real world robotics: no hands, no touch input, no audio input, no 50-dimensional continuous motor actions."

## [Deep Reinforcement Learning from Human Preferences](https://arxiv.org/pdf/1706.03741.pdf)

### Abstract

Train RL agent without reward function providing human feedback on less than 1% of agent's interations wiht the environment. Train complex novel behaviours with about an hour of human time.

Authors approach is to learn a reward function from human feedback and then to optimize that reward function. This basic approach has been considered previously, but we confront the challenges involved in scaling it up to modern deep RL and demonstrate by far the most complex behaviors yet learned from human feedback.

![deep_reinforcement_learning_from_human_preferences_fig0](images/deep_reinforcement_learning_from_human_preferences_fig0.png)

__Speaking of behavioural cloning:__

If we have demonstrations of the desired task, we can extract a reward function using inverse reinforcement learning (Ng and Russell, 2000). This reward function can then be used to train an agent with reinforcement learning.  More directly, we can use imitation learning to clone the demonstrated behavior. However, these approaches are not directly applicable to behaviors that are difficult for humans to demonstrate (such as controlling a robot with many degrees of freedom but very non-human morphology).

### Up-sides

* Tested in physics simulator MuJoCo as well as on Atari games.
* No need of reward function
* Only 1% of agents interactions needed for training
* Experiment details provided
* Unofficial. Code [available](https://github.com/nottombrown/rl-teacher). Tensorflow. Python 3.x.

### Useful for us

* It’s not clear how to construct a suitable reward function, which will need to be a function of the robot’s sensors. We could try to design a simple reward function that approximately captures the intended behavior,  but this will often result in behavior that optimizes our reward function without actually satisfying our preferences.

### Challenges

* Algortihm is tested in physics simulator however there are no proof that this approach would scale for real life scenarios such as robotics arm
* For real life scenarios this would be that learning should happen in real time with human feedback

## [Mechanical Design of a Cartesian Manipulator for Warehouse Pick and Place](http://juxi.net/papers/ACRV-TR-2017-02.pdf)

### Abstract

Robotic manipulations and grasping in cluttered and unstructured environments is a current challenge for robotics. One of main challenges associated with these difficult robotic manipulations tasks is the motion palling and control problem for multi-DoF (Dgree of Freedom). This paper presents the design and  performance evaluation of a novel low cost Cartesian manipulator, Cartman who  took  first  place  in  the  Amazon Robotics  Challenge  2017. Cost is < 10000 AUD

![cartman](images/cartman.png)

Cartman, a cartesian manipulator for pick and place applications. Cartman is composed of six degrees of freedom; three prismatic joints which form the x, y and z axes and three revolute joints and a multi-modal end effector for grasping a large range of items.

### Up-sides

* The complete design will be open sourced and will be available at the time of ICRA 2018
* Autonomously pick and place a variety of household itemm

### Useful for us

* Design
* They are using ROS (Robotic Operating System)

### Challenges

* Cost
* Size

## [Neural Task Programming Learning to Generalize Across Hierarchical Tasks](https://arxiv.org/abs/1710.01813?utm_campaign=Revue%20newsletter&utm_medium=Newsletter&utm_source=The%20Wild%20Week%20in%20AI)

### Abstract
In this work, we propose a novel robot learning framework called Neural Task Programming (NTP), which bridges the idea of few-shot learning from demonstration and neural program induction. NTP takes as input a task specification (e.g., video demonstration of a task) and recursively decomposes it into finer sub-task specifications. These specifications are fed to a hierarchical neural program, where bottom-level programs are callable subroutines that interact with the environment. We validate our method in three robot manipulation tasks. NTP achieves strong generalization across sequential tasks that exhibit hierarchal and compositional structures. The experimental results show that NTP learns to generalize well towards unseen tasks with increasing lengths, variable topologies, and changing objectives.

## [Rainbow: Combining Improvements in Deep Reinforcement Learning](https://arxiv.org/abs/1710.02298?utm_campaign=Revue%20newsletter&utm_medium=Newsletter&utm_source=The%20Wild%20Week%20in%20AI)

### Abstract
The deep reinforcement learning community has made several independent improvements to the DQN algorithm. However, it is unclear which of these extensions are complementary and can be fruitfully combined. This paper examines six extensions to the DQN algorithm and empirically studies their combination. Our experiments show that the combination provides state-of-the-art performance on the Atari 2600 benchmark, both in terms of data efficiency and final performance. We also provide results from a detailed ablation study that shows the contribution of each component to overall performance.

## [Learning to Push by Grasping: Using multiple tasks for effective learning](https://arxiv.org/pdf/1609.09025.pdf)

### Abstract
Recently, end-to-end learning frameworks are gaining prevalence in the field of robot control. These frameworks input states/images and directly predict the torques or the action parameters. However, these approaches are often critiqued due to their huge data requirements for learning a task. The argument of the difficulty in scalability to multiple tasks is well founded, since training these tasks often require hundreds or thousands of examples. But do end-to-end approaches need to learn a unique model for every task? Intuitively, it seems that sharing across tasks should help since all tasks require some common understanding of the environment. In this paper, we attempt to take the next step in data-driven end-to-end learning frameworks: move from the realm of task-specific models to joint learning of multiple robot tasks. In an astonishing result we show that models with multi-task learning tend to perform better than task-specific models trained with same amounts of data. For example, a deep-network learned with 2.5K grasp and 2.5K push examples performs better on grasping than a network trained on 5K grasp examples. 