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
2. From Docker image apple_picking_robot directory run `bash ros/run_apple_picker.sh` (open to see comments)

If you experience missing package error feel free to google and install it. Usually you should be able to install using `sudo apt-get package install ...` command line.

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
