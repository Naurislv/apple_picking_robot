# Robotic Operating System

It is not operating system but more like software framework to simplify robot development which can be installed in Docker or Linux (Ubuntu recommended) operating system.

Available lecture videos about ROS can be downloaded [here](https://www.dropbox.com/sh/54lppt8w7pfbg90/AAAiSUkj0A7eIN2Ap1UbOyB9a?dl=0).

# Basics

## Motivation

Many components of robotic systems may researchers, engineers and practitioners built from scratch over and over because of that ROS idea was born. Instead of repeating those steps ROS tools and environment right away. Not to mention huge community with tons of code available.

## ROS Components

Robot may be Drone, Car, Robotics arm or other and they have similar basic architecture.

![ros_project_architecture](images/ros_project_architecture.png)

### 1. ROS Master

Contains all necessary information of all active ROS nodes on system which allows other nodes to discover each other as well as parameters (configuration parameters).

### 2. Nodes

Entities of ROS blocks which contains certain functionality.

### 3. Messages

ROS support different types of messages which could be send between nodes such as Physical Quantities (Positions, Velocities, Accelerations, ..), Sensor Readings (Laser Scans, Images, Point Clouds, Measurements, ..).

Number of message types available are more than 200 but you can define your own message type if needed however it is recommended to search for already defined message types before defining your own.

### 4. Topics

Pub-Sub architecture.

Nodes can share messages with one another using Topics which is named bus e.g. /my_topic.

Node sending message to topic is called __Publishing___.
Node receiving message from topic is called __Subscribing__.

Messages are send directly from node to node without interacting with ROS Master.

### 5. Services

Request-Response architecture.

Similar to topics except can be called when needed instead of continuously publishing or subscribing to a topic which may result in less overhead.

Does not use publisher or subscriber like Topics but instead create service at a node by defining request and response.

## Compute Graph

Is a graph consisting of all Nodes, Services and Topics to represent systems architecture.

More info: http://wiki.ros.org/rqt_graph

## Debugging

Displaying images using rqt:

1. Run your ROS software
2. Run `roscore` from terminal, if it is not already running
3. Run `rqt_image_view` from terminal. GUI should pop up and then just choose right topic.

## Setup ROS

There are many tutorials how to setup ROS so we will not go into details but rather point out some issues we had.

1. Don't give up on first failure. Keep trying and looking for an answer. It should work but at the start it may be frustrating. We've all been there.
2. ROS using Python 2 by default and from our experience it should stay in that way. If you are using Python 3 then ROS may not run correctly as you must use Python 2 defined in System Paths. It's worth checking ~/.bashrc for that.
3. Virtual Machines are great but won't be able to GPUs so before creating Virtual Machine make sure you won't need GPU in future.
4. Rather than setting up individual environments for ROS use Git so environment for all would be same configure.

## Catkin workspace

### Catkin packages

ROS software is organized and distributed into packages, which are directories that might contain source code for ROS nodes, libraries, datasets, and more. Each package also contains a file with build instructions - the CMakeLists.txt file - and a package.xml file with information about the package. Packages enable ROS users to organize useful functionality in a convenient and reusable format.

### Catkin workspaces

A catkin workspace is a top-level directory where you build, install, and modify catkin packages. The workspace contains all of the packages for your project, along with several other directories for the catkin system to use when building executables and other targets from your source code.

### Roslaunch

When you want to start a ROS software you usually start it with launch file like this: `roslaunch launch/file.launch`. This brings up ROS loop and start all nodes described in launch file. These launch files usually bring up a set of nodes for the package that provide some aggregate functionality. This launch file can point to single node launch files and configuration files.

### Adding new package

In Catkin workspace new package mean new ROS node.

To add new package you must go through following steps (hello world example):

1. Define the path to package `.launch` file in `./ros/launch/project.launch` for example:

    ```
    <?xml version="1.0"?>
    <launch>
        <!--Hello World node -->
        <include file="$(find hello_world)/launch/hello_world.launch"/>
    </launch>P
    ```

2. Create `hello_world` package directory: `./ros/src/hello_world/`
3. Create `launch` directory in `hello_world`: `./ros/src/hello_world/launch`
4. Add simple script in `hello_world.py` which you want to be run in `./ros/src/hello_world/`:

   __!NB__ Every single python script must be executable (change permission for script in your file system) and start with `#!` declaration at the top.

    ```
    #!/usr/bin/env python

    """Simple ROS Catkin workspace package (node) example."""

    # Standard imports
    import time

    # Dependency imports
    import rospy

    class HelloWorld(object):
        """Traffic Light detection and classification. Results publishing to ROS nodes."""

        def __init__(self):
            rospy.init_node('hello_world')
            rospy.loginfo('Init hello_world')
            print('Hello Apple Picking Team. Congratulations for running ROS project for first time.')

            self.loop()

        def loop(self):
            """
                Loop function to publish waypoints for car to follow
            :return:
            """
            rate = rospy.Rate(10) # 10Hz

            i = 0
            while not rospy.is_shutdown():
                i += 1
                rospy.loginfo('Loop iter: %d', i)
                rate.sleep()

    if __name__ == '__main__':
        try:
            HelloWorld()
        except rospy.ROSInterruptException:
            rospy.logerr('Could not start hello_world node.')
    ```

4. Add `hello_world.launch` in `./ros/src/hello_world/launch` and define it to run `hello_world.py`:

    ```
    <?xml version="1.0"?>
    <launch>
        <node pkg="hello_world" type="hello_world.py" name="hello_world" output="screen" cwd="node"/>
    </launch>
    ```

5. Add `package.xml` in `./ros/src/hello_world/` and describe package (note that most lines are commented):

    Here you can define which packages this package depends on but not only, please read through example. Packages provided here are already existing in ROS but you also can define here that hello_world package depends on e.g. goodby_world package in same src directory.

    ```
    <?xml version="1.0"?>
    <package>
      <name>hello_world</name>
      <version>0.0.0</version>
      <description>Hello world package</description>

      <!-- One maintainer tag required, multiple allowed, one person per tag -->
      <!-- Example:  -->
      <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
      <maintainer email="yousuf@todo.todo">yousuf</maintainer>


      <!-- One license tag required, multiple allowed, one license per tag -->
      <!-- Commonly used license strings: -->
      <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
      <license>TODO</license>


      <!-- Url tags are optional, but multiple are allowed, one per tag -->
      <!-- Optional attribute type can be: website, bugtracker, or repository -->
      <!-- Example: -->
      <!-- <url type="website">http://wiki.ros.org/waypoint_loader</url> -->


      <!-- Author tags are optional, multiple are allowed, one per tag -->
      <!-- Authors do not have to be maintainers, but could be -->
      <!-- Example: -->
      <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


      <!-- The *_depend tags are used to specify dependencies -->
      <!-- Dependencies can be catkin packages or system dependencies -->
      <!-- Examples: -->
      <!-- Use build_depend for packages you need at compile time: -->
      <!--   <build_depend>message_generation</build_depend> -->
      <!-- Use buildtool_depend for build tool packages: -->
      <!--   <buildtool_depend>catkin</buildtool_depend> -->
      <!-- Use run_depend for packages you need at runtime: -->
      <!--   <run_depend>message_runtime</run_depend> -->
      <!-- Use test_depend for packages you need only for testing: -->
      <!--   <test_depend>gtest</test_depend> -->
      <buildtool_depend>catkin</buildtool_depend>
      <build_depend>geometry_msgs</build_depend>
      <build_depend>roscpp</build_depend>
      <build_depend>rospy</build_depend>
      <build_depend>sensor_msgs</build_depend>
      <build_depend>std_msgs</build_depend>
      <build_depend>styx_msgs</build_depend>
      <build_depend>waypoint_updater</build_depend>
      <run_depend>geometry_msgs</run_depend>
      <run_depend>roscpp</run_depend>
      <run_depend>rospy</run_depend>
      <run_depend>sensor_msgs</run_depend>
      <run_depend>std_msgs</run_depend>

      <!-- The export tag contains other, unspecified, tags -->
      <export>
        <!-- Other tools can request additional information be placed here -->

      </export>
    </package>
    ```

6. Add `CMakeLists.txt` in `./ros/src/hello_world` (note that most lines are commented):

    Here you can define which packages this package depends on but not only, please read through example. Packages provided here are already existing in ROS but you also can define here that hello_world package depends on e.g. goodby_world package in same src directory.

    ```
    cmake_minimum_required(VERSION 2.8.3)
    project(tl_detector)

    ## Add support for C++11, supported in ROS Kinetic and newer
    # add_definitions(-std=c++11)

    ## Find catkin macros and libraries
    ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
    ## is used, also find other catkin packages
    find_package(catkin REQUIRED COMPONENTS
      geometry_msgs
      roscpp
      rospy
      sensor_msgs
      std_msgs
    )

    ## System dependencies are found with CMake's conventions
    # find_package(Boost REQUIRED COMPONENTS system)


    ## Uncomment this if the package has a setup.py. This macro ensures
    ## modules and global scripts declared therein get installed
    ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
    # catkin_python_setup()

    ################################################
    ## Declare ROS messages, services and actions ##
    ################################################

    ## To declare and build messages, services or actions from within this
    ## package, follow these steps:
    ## * Let MSG_DEP_SET be the set of packages whose message types you use in
    ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
    ## * In the file package.xml:
    ##   * add a build_depend tag for "message_generation"
    ##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
    ##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
    ##     but can be declared for certainty nonetheless:
    ##     * add a run_depend tag for "message_runtime"
    ## * In this file (CMakeLists.txt):
    ##   * add "message_generation" and every package in MSG_DEP_SET to
    ##     find_package(catkin REQUIRED COMPONENTS ...)
    ##   * add "message_runtime" and every package in MSG_DEP_SET to
    ##     catkin_package(CATKIN_DEPENDS ...)
    ##   * uncomment the add_*_files sections below as needed
    ##     and list every .msg/.srv/.action file to be processed
    ##   * uncomment the generate_messages entry below
    ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

    ## Generate messages in the 'msg' folder
    # add_message_files(
    #   FILES
    #   Message1.msg
    #   Message2.msg
    # )

    ## Generate services in the 'srv' folder
    # add_service_files(
    #   FILES
    #   Service1.srv
    #   Service2.srv
    # )

    ## Generate actions in the 'action' folder
    # add_action_files(
    #   FILES
    #   Action1.action
    #   Action2.action
    # )

    ## Generate added messages and services with any dependencies listed here
    # generate_messages(
    #   DEPENDENCIES
    #   geometry_msgs#   sensor_msgs#   std_msgs#   styx_msgs
    # )

    ################################################
    ## Declare ROS dynamic reconfigure parameters ##
    ################################################

    ## To declare and build dynamic reconfigure parameters within this
    ## package, follow these steps:
    ## * In the file package.xml:
    ##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
    ## * In this file (CMakeLists.txt):
    ##   * add "dynamic_reconfigure" to
    ##     find_package(catkin REQUIRED COMPONENTS ...)
    ##   * uncomment the "generate_dynamic_reconfigure_options" section below
    ##     and list every .cfg file to be processed

    ## Generate dynamic reconfigure parameters in the 'cfg' folder
    # generate_dynamic_reconfigure_options(
    #   cfg/DynReconf1.cfg
    #   cfg/DynReconf2.cfg
    # )

    ###################################
    ## catkin specific configuration ##
    ###################################
    ## The catkin_package macro generates cmake config files for your package
    ## Declare things to be passed to dependent projects
    ## INCLUDE_DIRS: uncomment this if you package contains header files
    ## LIBRARIES: libraries you create in this project that dependent projects also need
    ## CATKIN_DEPENDS: catkin_packages dependent projects also need
    ## DEPENDS: system dependencies of this project that dependent projects also need
    catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES waypoint_loader
    #  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs styx_msgs
    #  DEPENDS system_lib
    )

    ###########
    ## Build ##
    ###########

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    # include_directories(include)
    include_directories(
      ${catkin_INCLUDE_DIRS}
    )

    ## Declare a C++ library
    # add_library(${PROJECT_NAME}
    #   src/${PROJECT_NAME}/waypoint_loader.cpp
    # )

    ## Add cmake target dependencies of the library
    ## as an example, code may need to be generated before libraries
    ## either from message generation or dynamic reconfigure
    # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    ## Declare a C++ executable
    ## With catkin_make all packages are built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    # add_executable(${PROJECT_NAME}_node src/waypoint_loader_node.cpp)

    ## Rename C++ executable without prefix
    ## The above recommended prefix causes long target names, the following renames the
    ## target back to the shorter version for ease of user use
    ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
    # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

    ## Add cmake target dependencies of the executable
    ## same as for the library above
    # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    ## Specify libraries to link a library or executable target against
    # target_link_libraries(${PROJECT_NAME}_node
    #   ${catkin_LIBRARIES}
    # )

    #############
    ## Install ##
    #############

    # all install targets should use catkin DESTINATION variables
    # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

    ## Mark executable scripts (Python etc.) for installation
    ## in contrast to setup.py, you can choose the destination
    # install(PROGRAMS
    #   scripts/my_python_script
    #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    # )

    ## Mark executables and/or libraries for installation
    # install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    # )

    ## Mark cpp header files for installation
    # install(DIRECTORY include/${PROJECT_NAME}/
    #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    #   FILES_MATCHING PATTERN "*.h"
    #   PATTERN ".svn" EXCLUDE
    # )

    ## Mark other files for installation (e.g. launch and bag files, etc.)
    # install(FILES
    #   # myfile1
    #   # myfile2
    #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    # )

    #############
    ## Testing ##
    #############

    ## Add gtest based cpp test target and link libraries
    # catkin_add_gtest(${PROJECT_NAME}-test test/test_waypoint_loader.cpp)
    # if(TARGET ${PROJECT_NAME}-test)
    #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
    # endif()

    ## Add folders to be run by python nosetests
    # catkin_add_nosetests(test)
    ```
