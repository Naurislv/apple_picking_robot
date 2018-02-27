# [Docker](https://www.docker.com/)

Here we cover basics of how to use Docker on any OS and to set-up it with our docker image.

As we are working in team we should all be able to reproduce results. Often it's hard to reproduce results because each of team member is using different OS (and version), different environment set-up etc. [Docker](https://www.docker.com/) to rescue.

Basically Docker is pre-installed image which can be run on any OS. It's like light weight VirtualBox. Good thing about Docker is that it works like Git. You can make changes and then commit them and upload for everyone to pull it with all history.

# First time run

## Installing Docker on your machine

[Download](https://docs.docker.com/engine/installation/) right Docker version for your machine and follow instructions to install it. After Docker is installed it's wise to inspect settings. If Docker is not running already then run Docker, it will automatically allocate default (configuration) resources. Open preferences and carefully inspect them. Perhaps the most important section is called "Advanced", there you can see resources Docker will use on your machine as well as path where it will store all pulled images.

See next section on how to setup Docker with our pre-installed image. Note that ROS already provide Ubuntu with pre-installed ROS. See [tutorial](http://wiki.ros.org/docker/Tutorials/Docker) for more info. [This](https://www.youtube.com/watch?v=9xqekKwzmV8) YouTube video also might be helpful. In our project however we have already pre-installed everything so you dont need to install this ROS package.

# Seting up ROS Docker image with VNC support for GUI

Origin Docker image used: https://hub.docker.com/r/ct2034/vnc-ros-kinetic-full/

1. Download and run: `docker run -it -p 6080:80 ct2034/vnc-ros-kinetic-full`
2. Open in browser: http://127.0.0.1:6080 and open terminal
3. Update, upgrade and install packages

    ```
    sudo apt-get update && sudo apt-get upgrade
    sudo apt-get install nano wget
    sudo apt-get install ros-kinetic-turtlebot*

    # Reference: http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0#Alternativeinstallation:step-by-step
    
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    cat /etc/apt/sources.list.d/gazebo-stable.list
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install gazebo7

    pip install --upgrade pip
    pip install imutils tensorflow
    ```
4. Setup apple_picker repository

    ```
    cd /home
    git clone https://github.com/Naurislv/apple_picking_robot.git
    ```

# Docker Help

As previously mention images in Docker works similar to repositories in Git. You pull them, you run them, make changes and then commit them. All your commits and versions are stored and can be seen using `docker images` CLI command.

## Running, printing and removing images and containers

To run docker image you can use either Repository name or Image Id which (from `docker images` output). Example: `docker run -it ros`.

Once image is running Docker creates container. All running containers can be seen by executing `docker ps`. If you want to see also stopped containers execute `docker ps -a`.

From time to time you may want to remove images and / or containers. To remove single image run `docker rmi IMAGE_ID`, to remove all containers run `docker rm $(docker ps -aq)`.

Connect with additinal bash terminal to container: `docker exec -it CONTAINER_ID bash`

## Committing changes to images

Here is official documentation and example of [commit command for Docker](https://docs.docker.com/engine/reference/commandline/commit/)

Example: `docker commit 1222f43418c4 ros/apple_picker:v0.2`

## Pushing changes

[Tutorial](https://docs.docker.com/docker-cloud/builds/push-images/)

Example:

1. `docker tag ros/apple_picker:v0.3 naurislv/apple_picking_robot:latest`
2. `docker push naurislv/apple_picking_robot:latest`

## Useful Docker commands

* View all images: `docker images`
* View all containers: `docker ps -a`
* Run image with with terminal: `docker run -it IMAGE_ID`
* Start container: `docker start CONTAINER_ID`
* Open bash terminal on running container: `docker exec -i -t CONTAINER_ID /bin/bash`
* Remove all containers: `docker rm $(docker ps -a -q)`