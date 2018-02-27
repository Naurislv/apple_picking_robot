# Gazebo

[Gazebo](http://wiki.ros.org/gazebo) is [ROS](http://www.ros.org/) virtual environment packge used for robotics projects.

# Turtlebot

[Turtlebot](http://wiki.ros.org/turtlebot_gazebo) is basic implementation of controllable robot in Gazebo which can be extened and used for any kind of projects.

## Innstall turtlebot3 simulator

1. You should clone following repositories to catkin_WS source directory:

   * Tutlebot3 the main repo: `git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`
   * Tutlebot3 repo for message exchanging: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`
   * Tutlebot3 repo for simulatots: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`
   * Tutlebot3 repo for keyboard connection: `git clone https://github.com/ros-teleop/teleop_twist_keyboard.git`
   * Tutlebot3 repo for joystick connection: `git clone https://github.com/ros-teleop/teleop_twist_joy.git`

2. After that compile: `catkin_make`

You may experience error, but it doesn't affect our simulation. See error below:

```
CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "amcl" with any of
  the following names:

    amclConfig.cmake
    amcl-config.cmake

  Add the installation prefix of "amcl" to CMAKE_PREFIX_PATH or set
  "amcl_DIR" to a directory containing one of the above files.  If "amcl"
  provides a separate development package or SDK, be sure it has been
  installed.
Call Stack (most recent call first):
  turtlebot3/turtlebot3_navigation/CMakeLists.txt:10 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/student/catkin_ws/build/CMakeFiles/CMakeOutput.log".
See also "/home/student/catkin_ws/build/CMakeFiles/CMakeError.log".
Invoking "cmake" failed
```

Make sure you have installed all above packages dependences we have, use for example:

  * `rosdep check turtlebot3_gazebo`
  * install all dependecies: `rosdep -i install  turtlebot3_gazebo`

3. Activate you developing space:

`source ¬/devel/setup.bash`

4. Don't forget your  turtlebot model. Burger is more suitable to IRobot.

`export TURTLEBOT3_MODEL="burger"`

5. Launch your world package:

`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

For using keyboard you should open another terminal and do the same steps from catkin_make compiling. Instead world launching just execute this:

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

We can add a button which 'picks an apple' in this file I think.

![Turtlebot burger is in the gazebo world](images/Turtlebot3_burger_gazebo_simulation.png)




