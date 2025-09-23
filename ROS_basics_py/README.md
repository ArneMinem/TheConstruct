**Date:** 22/09/25

**Author:** Arne JACOBS

# ROS basics in 5 days (Pyhton)

In this course we learn how to use ROS with Python.

## Table of Contents
1. [Ways to communicate with your robot](#ways-to-communicate-with-your-robot)
  - [Topics](#topics)
  - [Services](#services)
  - [Actions](#actions)
2. [Visualisation](#visualisation)
  - [Rviz](#rviz)
  - [Rqt](#rqt)
3. [Launch](#launch)
  - [Launching](#launching)
  - [Launch file](#launch-file)
4. [Package construction](#package-construction)
  - [Compile a package](#compile-a-package)
  - [Creating a ROS package](#creating-a-ros-package)
  - [Creating a ROS program](#creating-a-ros-program)
5. [Node](#node)
6. [Publishers and Subscribers](#publishers-and-subscribers)
  - [Publisher](#publisher)
  - [Message types](#message-types)
  - [Subscriber](#subscriber)
7. [Param Server](#param-server)
8. [Roscore](#roscore)
9. [Environment Variables](#environment-variables)


## Ways to communicate with your robot

### Topics

Publish in topics. Example: Publish commands in cmd_vel which the robot will read and follow 

`roslaunch publisher_example move.launch`

`roslaunch publisher_example stop.launch`

### Services

Create a service server in which there are multiple services that anyone can use. When created and running, call services. 

`roslaunch service_demo service_launch.launch`

`rosservice call /service_demo "{}"`

### Actions

Create an action server in which there are multiple actions that anyone can use. When created and running, call actions. The difference with services is that actions can be called anytime and do feedback. 

`roslaunch action_demo action_launch.launch`

`roslaunch action_demo_client client_launch.launch`

## Visualisation

### Rviz

`rosrun rviz rviz`

### Rqt

`???`

## Launch

### Launching

Usual structure of the command: 

`roslaunch <package_name> <launch_file>` 

Example for Teleoperation: 

`roslaunch turtlebot_teleop keyboard_teleop.launch`

### Launch file 

```
<launch> 
  <!-- turtlebot_teleop_key already has its own built in velocity smoother --> 
  <node pkg="turtlebot_teleop" type="turtlebot_teleop_key.py" name="turtlebot_teleop_keyboard"  output="screen"> 
    <param name="scale_linear" value="0.5" type="double"/> 
    <param name="scale_angular" value="1.5" type="double"/> 
    <remap from="turtlebot_teleop_keyboard/cmd_vel" to="/cmd_vel"/>   <!-- cmd_vel_mux/input/teleop"/--> 
  </node> 
</launch>
```

All launch files are contained within a <launch> tag. Inside that tag, you can see a <node> tag, where we specify the following parameters: 

- **pkg**="package_name" # Name of the package that contains the code of the ROS program to execute 

- **type**="python_file_name.py" # Name of the program file that we want to execute 

- **name**="node_name" # Name of the ROS node that will launch our Python file 

- **output**="type_of_output" # Through which channel you will print the output of the Python file 

## Package construction 

All those files in the package are organized with the following structure: 

- **launch** folder: Contains launch files 

- **src** folder: Source files (cpp, python) 

- **CMakeLists.txt**: List of cmake rules for compilation 

- **package.xml**: Package information and dependencies 

To go to any ROS package, ROS gives you a command named roscd. When typing: `roscd <package_name>`

### Compile a package 

Command to use, ALWAYS IN catkin_ws DIRECTORY: 

`catkin_make` 

For specific compilation of a package and its dependencies: 

`catkin_make --only-pkg-with-deps <package_name>` 

Always source afterwards: 

`source devel/setup.bash` 

### Creating a ROS package 

First, go to `/home/user/catkin_ws` with  `roscd` + `cd ..` 

Then go in src and create the package with: 

`catkin_create_pkg <package_name> <package_dependecies>`

Ex: `catkin_create_pkg my_package rospy` or `catkin_create_pkg my_publisher_example_pkg rospy std_msgs`

Useful: 

`rospack list`

`rospack list | grep my_package`

### Creating a ROS program 

Can be done with a right-click in the repository 

Useful: 

`chmod +x name_of_the_file.py`  Useful if file created without execution permissions 

`rospack profile`  Useful to have ROS refresh its package list

## Node 

`rosnode list`

`rosnode info <node_name>`     (ex: `rosnode info /ObiWan`)

To create a Node with a 2Hz rate (2 times per second) in a Python code: 

```
#! /usr/bin/env python 
 
import rospy 
 
rospy.init_node("ObiWan") 
rate = rospy.Rate(2)               # We create a Rate object of 2Hz 
while not rospy.is_shutdown():     # Endless loop until Ctrl + C 
   print("Help me Obi-Wan Kenobi, you're my only hope") 
   rate.sleep()                    # We sleep the needed time to maintain the Rate fixed above 
     
# This program creates an endless loop that repeats itself 2 times per second (2Hz) until somebody presses Ctrl + C 
# in the terminal 
```

## Publishers and Subscribers

### Publisher

Publisher example: 

```
#! /usr/bin/env python

# Import the Python library for ROS
import rospy
# Import the Int32 message from the std_msgs package
from std_msgs.msg import Int32             


# Initiate a Node named 'topic_publisher'
rospy.init_node('topic_publisher')

# Create a Publisher object, that will publish on the /counter topic
# messages of type Int32
pub = rospy.Publisher('/counter', Int32, queue_size=1)    
                                           
# Set a publish rate of 2 Hz
rate = rospy.Rate(2)
# Create a variable of type Int32
count = Int32()
# Initialize 'count' variable
count.data = 0                             

# Create a loop that will go until someone stops the program execution
while not rospy.is_shutdown():
  # Publish the message within the 'count' variable
  pub.publish(count)
  # Increment 'count' variable
  count.data += 1
  # Make sure the publish rate maintains at 2 Hz
  rate.sleep()                             
```

Useful commands:

`rostopic list` to list all topics

`rostopic echo /counter` to display the messages being published on the /counter topic

`rostopic echo /counter -n1` to display the latest message published on the /counter topic

`rostopic info /counter` to get information about the /counter topic

`rostopic -h` to get help about the rostopic command

### Message types

Messages are defined in .msg files, which are located inside a msg directory of a package. (You can find them with `roscd std_msgs/msg`)

`rosmsg show std_msgs/Int32` to see the structure of the Int32 message type

- std_msgs: standard message types, like Int32, String, Float32, Bool, etc.
- geometry_msgs: message types for geometric data, like Point, Pose, Twist, etc.
- sensor_msgs: message types for sensor data, like Image, LaserScan, Imu,

Example of Twist message type:

```
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)

var = Twist()

var.linear.x = 0.5   
var.linear.y = 0.0
var.linear.z = 0.0

var.angular.x = 0.0
var.angular.y = 0.0
var.angular.z = 0.3 

while not rospy.is_shutdown(): 
  pub.publish(var)
  rate.sleep()
```

### Subscriber

Subscriber example: 

```
#! /usr/bin/env python
```


## Param Server 

`rosparam list` 

`rosparam get <parameter_name>` 

`rosparam set <parameter_name> <value>`

## Roscore 

It is always necessary to have a roscore running. It is the main process that manages all of the ROS system. 

`roscore`

## Environment Variables 

`export | grep ROS`

In The Construct we can use: `rosenv`