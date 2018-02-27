# Apple picking robot

Creating autonomous robot for given size apples pitching in the controlled indoor environment to gain and share the experience.

## Project structure

* [data/](data/) used throughout project. Usually we ignore data stored there from .git perspective.
* [docs/](docs/) project documentations. All topics we want to be documented. See [Docs](#docs) table of content.
* Scientific [papers/](papers/) and [reviews](papers/PaperReview.md). Usually you give and share papers using url but there are some papers which we may want to store in repository.
* [presentations/](presentations/) by team members.
* [ros/](ros/) (Robot Operating System) used for inference with Virtual Environments or Robots
* [experiments/](experiments/) consists of different experiments such as training neural network, preparing datasets etc.
* [weekly_retrospects/](weekly_retrospects/). Short descriptions of what we've talked in weekly meetings.

## Run

1. Follow [first time run](docs/Docker.md) instructions to install and setup pre-installed Docker image.
2. Running project.
    * First navigate to ros directory: `cd ros`
    * Run `./run_apple_picker.sh --help` to see all possible input arguments:

    ```
    Apple Picker

    [options] application [arguments]

    options:

    -h,  --help                        show brief help
    -rv, --rviz=bool                   if true then run ROS rviz GUI
    -r,  --rqt=bool                    if true then run ROS rqt_image_view
    -c,  --camera=bool                 if true then physical camera connected and no virtual env will be launched
    -su, --scene_understanding=bool    if true then scene_understanding will be launched
    -kb, --keyboard=bool               if true then run keyboard package for controlig Gazebo
    -w,  --world=WORLD                 choose turtlebot world. empty | original | dbaby
    -gg, --gazebo_gui=bool             if true then run gazebo GUI
    -yt, --youtube=LINK                download youtube (LINK) video and copy it experiment directory

    Example: ./run_apple_picker --rviz=true
    Example: ./run_apple_picker -rv true
    ```
    * Run script with arguments e.g.: `./run_apple_picker.sh -r true -kb true -w dbaby`

## Docs

1. [Robot (real and virtual environment) software and hardware architectures](docs/ArchitectureProposal.md) and related paper reviews.
2. [Intro to ROS](docs/ROS.md)
3. How to use [Docker](docs/Docker.md) for this project.
4. [UnrealEngine4](docs/UnrealEngine4.md)
5. How to setup [Gazebo](docs/Gazebo.md)

# The Team

In Progress: Create table of all team members

|     Image              |     Role      |      Name      |    Location   | LinkedIn    |     email   |
|------------------------|---------------|----------------|---------------|-------------|-------------|
| <img src="./images/nauris_dorbe.jpg" alt="Nauris Dorbe" width="150" height="150"> |__ML, Autonomy__| Nauris Dorbe | Latvia, Riga | [Nauris](https://www.linkedin.com/in/naurisdorbe) | <naurisdorbe@gmail.com> |

# Our story

## [NIPS](https://nips.cc/) (Neural Information Processing Systems) conference 2017

We've been there with [dBaby](https://github.com/LUMII-AILab/dBaby)
