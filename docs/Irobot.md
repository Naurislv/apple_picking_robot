Based on this repo: 

https://github.com/CentroEPiaggio/irobotcreate2ros.git


---
How to install

rosdep -i install irobotcreate2

---
How to launch

source devel/setup.bash
export iRobot_model="iRobot" 


roslaunch irobotcreate2 irobot_dbaby_world.launch

---

If you change something in URDF file, before compiling you should recreate a new SDF model:
cd ¬/irobotcreate2ros/model

source generate_model.sh


./run_apple_picker.sh -r true -w  dbaby -gg true -t true

---

bumper sensor topic:
/irobot_bumper/states
---
new topic for camera:
/iRobot/camera/image_raw
