# Gazebo

TBD

## Apple world

How to launch an apple world:

1.Open new termnal

2.Open catkin_ws folder

3.Complile your packages with 'catkin_make'

4.Activate you developing space:

`source ¬/devel/setup.bash`

5.Define your  turtlebot model. Burger is more suitable to IRobot.

`export TURTLEBOT3_MODEL="burger"`

6.Launch your world package:

`roslaunch turtlebot3_gazebo turtlebot3_apple_dbaby_world.launch`


## Keyboard operation module

How to launch an operation module:

1.Open new termnal

2.Activate you developing space:

`source ¬/devel/setup.bash`

3.Be sure that roscore is working in the separate terminal

4.Launch your kewboard package:

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

5.For using keyboard the terminal with this package should be active.


## Camera module

How to launch a rqt_image_view module:

1.Open new termnal

2.Activate you developing space:

`source ¬/devel/setup.bash`

3.Be sure that roscore is working in the separate terminal

4.Just use a command:

`rqt_image_view`

5.Select the topic:

`/camera1/image_raw`


## Rviz module

How to launch a rviz module and see robot sensors result:

1.Open new termnal

2.Activate you developing space:

`source ¬/devel/setup.bash`

3.Define your  turtlebot model. Burger is more suitable to IRobot.

`export TURTLEBOT3_MODEL="burger"`

4.Launch your rviz package:

`roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch`

5.In camera section just select the topic:

`/camera1/image_raw`



