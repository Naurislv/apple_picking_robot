## Requirements

* Linux machine with NVIDIA GPU and drivers
* Docker
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker)
* [VirtualGL](https://www.virtualgl.org/) client
* X server on client machine

## Build

```
$ docker build -t apple-picking-robot-gazebo-nvidia-vgl .
```

## Run

1. SSH to target machine via [VirtualGL](https://www.virtualgl.org/) `vglconnect` (on macOS `/opt/VirtualGL/bin/vglconnect target-machine`)

2. Run gazebo

   ```
   $ ./run.sh -it apple-picking-robot-gazebo-nvidia-vgl vglrun roslaunch turtlebot3_gazebo turtlebot3_apple_dbaby_world.launch
   ```

   or execute bash shell

   ```
   $ ./run.sh -it apple-picking-robot-gazebo-nvidia-vgl vglrun /bin/bash
   ```

   Optionally you may add more `docker run` arguments placing them right after `run.sh`.


This will launch an internal Xorg server at headless display :0 where VirtualGL renders the OpenGL content
and forwards already rendered OpenGL content to remote X server connected via VirtualGL's `vglconnect` SSH tunnel.
