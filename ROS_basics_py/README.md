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
10. [Launch simulation](#launch-simulation)


## Ways to communicate with your robot

### Topics

Publish in topics. Example: Publish commands in cmd_vel which the robot will read and follow 

`roslaunch publisher_example move.launch`

`roslaunch publisher_example stop.launch`

### Services

Create a service server in which there are multiple services that anyone can use. When created and running, call services. 

`roslaunch service_demo service_launch.launch`

`rosservice call /service_demo "{}"`

Services are Synchronous. When your ROS program calls a service, your program can't continue until it receives a result from the service.

Use services when your program can't continue until it receives the result from the service.

`rosservice list` to list all services
`rosservice info /service_demo` to get information about the /service_demo service

Example with /exec_trajectory service:

`rosservice info /exec_trajectory`

The output is:

```
Node: /iri_wam_reproduce_trajectory
URI: rosrpc://3_xterm:34497
Type: iri_wam_reproduce_trajectory/ExecTraj
Args: file
```

- **Node:** It states the node that provides (has created) that service.
- **Type:** It refers to the kind of message used by this service. It has the same structure as topics do. It's always made of package_where_the_service_message_is_defined / Name_of_the_File_where_Service_message_is_defined. In this case, the package is iri_wam_reproduce_trajectory, and the file where the Service Message is defined is called ExecTraj.
- **Args:** Here you can find the arguments that this service takes when called. In this case, it only takes a trajectory file path stored in the variable called file.

For more information about the structure of the service message, you can use the command:

`rossrv show name_of_the_package/Name_of_the_Service_Message`

Example:

`rosservice info /trajectory_by_name

`rossrv show trajectory_by_name_srv/TrajByName`

#### Calling a service

To call a service, you use the command `rosservice call /name_of_the_service "arguments"`

You can autocomplete the structure of the arguments by typing `rosservice call /name_of_the_service` and then pressing the TAB key twice.

Example:

`rosservice call /trajectory_by_name [tab][tab]` gives `rosservice call /trajectory_by_name "traj_name: ''"`

You can then fill in the argument:

`rosservice call /trajectory_by_name "traj_name: 'get_food'"`

**Careful:**

- `rossrv list`

This shows all service types that are compiled and available in your ROS system.
Examples:

std_srvs/Empty

my_custom_srv_msg_pkg/MyCustomServiceMessage

These are like class definitions → they describe the format of the request and response messages.

- `rosservice list`

This shows all running service instances (servers) that are currently active.
Examples:

/gazebo/get_model_state

/my_service

These are like objects created from the class → they exist only when a node advertises them.

#### Service client program example

```
#! /usr/bin/env python

import rospy
# Import the service message used by the service /trajectory_by_name
from trajectory_by_name_srv.srv import TrajByName, TrajByNameRequest
import sys

# Initialise a ROS node with the name service_client
rospy.init_node('service_client')
# Wait for the service client /trajectory_by_name to be running
rospy.wait_for_service('/trajectory_by_name')
# Create the connection to the service
traj_by_name_service = rospy.ServiceProxy('/trajectory_by_name', TrajByName)
# Create an object of type TrajByNameRequest
traj_by_name_object = TrajByNameRequest()
# Fill the variable traj_name of this object with the desired value
traj_by_name_object.traj_name = "release_food"
# Send through the connection the name of the trajectory to be executed by the robot
result = traj_by_name_service(traj_by_name_object)
# Print the result given by the service called
print(result)
```

#### Service messages

Service message files are defined in .srv files, which are located inside a srv directory of a package. (You can find them with `roscd trajectory_by_name_srv/srv`)
While topic message files are defined in .msg files, located inside a msg directory of a package.
Instead of using `rosmsg show`, you have to use `rossrv show` to see the structure of the service message type.

A service message is always made of 2 parts, separated by a line with 3 dashes `---`.

The first part defines the arguments that the service receives when called. The **request**.

The second part defines the arguments that the service returns when it finishes its task. The **response**.

The **REQUEST** is the part of the service message that defines HOW you will do a call to your service. This means, what variables you will have to pass to the Service Server so that it is able to complete its task.

The **RESPONSE** is the part of the service message that defines HOW your service will respond after completing its functionality. If, for instance, it will return an string with a certaing message saying that everything went well, or if it will return nothing, etc...

Whenever a service message is compiled, three message objects are created:
- NameOfTheService: This is the object that you will use to create a connection to the service. It is used when you create a ServiceProxy object.
- NameOfTheServiceRequest: This is the object that you will use to create a variable of the type of the service request. It is used when you want to fill in the arguments that the service request needs.
- NameOfTheServiceResponse: This is the object that you will use to create a variable of the type of the service response. It is used when you want to fill in the arguments that the service response needs.

**Creating your own service message:**

1. Create a srv directory in your package: `mkdir srv`
2. Create a .srv file in the srv directory: `touch MyService.srv`
3. Edit the .srv file to define the service structure:
```
string words
---
int32 number_of_words
```

4. Modify CMakeLists.txt:

```
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
add_service_files(
  FILES
  MyService.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)
```

5. Modify package.xml:

Add the following lines:
```
<build_depend>message_generation</build_depend>
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```

#### Service Server example

```
#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.


def my_callback(request):
    print("My_callback has been called")
    return EmptyResponse() # the service Response class, in this case EmptyResponse
    #return MyServiceResponse(len(request.words.split())) 

rospy.init_node('service_server') 
my_service = rospy.Service('/my_service', Empty, my_callback) # create the Service called my_service with the defined callback
rospy.spin() # maintain the service open.
```

Bigger example to command bb8:
```
#! /usr/bin/env python

import rospy
# you import the service message python classes generated from Empty.srv
from std_srvs.srv import Empty, EmptyResponse 
from geometry_msgs.msg import Twist

def my_callback(request):
    print("My_callback has been called")

    cmd = Twist()
    cmd.linear.x = 0.5
    cmd.angular.z = 0.5

    rate = rospy.Rate(2)

    pub.publish(cmd)

    # the service Response class, in this case EmptyResponse
    return EmptyResponse()

rospy.init_node('bb8_move_in_circle_service_server')

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
my_service = rospy.Service('/move_bb8_in_circle', Empty, my_callback) # create the Service called move_bb8_in_circle with the defined callback
rospy.spin() # maintain the service open.
```

### Actions

Create an action server in which there are multiple actions that anyone can use. When created and running, call actions. The difference with services is that actions can be called anytime and do feedback. 

`roslaunch action_demo action_launch.launch`

`roslaunch action_demo_client client_launch.launch`

Actions are Asynchronous. It's like launching a new thread. When your ROS program calls an action, your program can perform other tasks while the action is being performed in another thread.

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

Other example of a launch file that calls another launch file: 

```
<launch>

  <include file="$(find iri_wam_reproduce_trajectory)/launch/start_service.launch"/>

  <node pkg ="iri_wam_aff_demo"
        type="iri_wam_aff_demo_node"
        name="iri_wam_aff_demo"
        output="screen">
  </node>

</launch>
```

The first part of the launch file calls another launch file called start_service.launch.

That launch file starts the node that provides the /execute_trajectory service. Note that it's using a special ROS launch file function to find the path of the package given.

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
- sensor_msgs: message types for sensor data, like Image, LaserScan, Imu, etc.

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

**Create your own message:**

1. Create a msg directory in your package: `mkdir msg`
2. Create a .msg file in the msg directory: `touch Age.msg`
3. Edit the .msg file to define the message structure:
```
float32 years
float32 months
float32 days
```

4. Modify CMakeLists.txt:

```
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Age.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)
```

5. Modify package.xml:

Add the following lines:
```
<build_depend>message_generation</build_depend>

<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```

Make a python program to publish the message:

```
#! /usr/bin/env python

import rospy
from your_package_name.msg import Age  # Replace 'your_package_name' with the actual package (here my_first_publisher_pkg)

rospy.init_node('age_publisher')
pub = rospy.Publisher('/age', Age, queue_size=1)
rate = rospy.Rate(2)

age = Age()
age.years = 5
age.months = 6
age.days = 7

while not rospy.is_shutdown():
    pub.publish(age)
    rate.sleep()
```

How to launch:
1. Compile the package: `catkin_make`
2. Source the workspace: `source devel/setup.bash`
3. Run the publisher node: `rosrun your_package_name your_script.py`

If it doesn't work remove build and devel folders and recompile.

If you want to see the message being published, use: `rostopic echo /age`

If this doesn't work do a `source devel/setup.bash` in the terminal where you want to see the message.


### Subscriber

Subscriber example: 

```
#! /usr/bin/env python

import rospy                                          
from std_msgs.msg import Int32 

def callback(msg):                                    # Define a function called 'callback' that receives a parameter 
                                                      # named 'msg'
  
    print (msg.data)                                  # Print the value 'data' inside the 'msg' parameter

rospy.init_node('topic_subscriber')                   # Initiate a Node called 'topic_subscriber'

sub = rospy.Subscriber('/counter', Int32, callback)   # Create a Subscriber object that will listen to the /counter
                                                      # topic and will cal the 'callback' function each time it reads
                                                      # something from the topic
rospy.spin()                                          # Create a loop that will keep the program in execution
```

Other example:
```
#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg): 
print ("Position: x = ", msg.pose.pose.position.x, ", y = ", msg.pose.pose.position.y, ", z = ", msg.pose.pose.position.z)
print ("Orientation: x = ", msg.pose.pose.orientation.x, ", y = ", msg.pose.pose.orientation.y, ", z = ", msg.pose.pose.orientation.z)

rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
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

## Launch simulation

`source /home/user/simulation_ws/devel/setup.bash && roslaunch realrobotlab main.launch`

## 
