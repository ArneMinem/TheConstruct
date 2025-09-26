# Create my custom service

## Create folder srv in the package folder and a file named MyService.srv in it.

```
mkdir srv
cd srv
touch MoveCircleCustom.srv
```
Edit the file to add the request and response fields:

```
int32 duration    # The time (in seconds) during which BB-8 will keep moving in circles
---
bool success      # Did it achieve it?
```

## Edit CMakeLists.txt to add the service file
```
cmake_minimum_required(VERSION 3.0.2)
project(unit_5_services)

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
 CATKIN_DEPENDS rospy message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
```
## Edit package.xml to add the service dependencies

```
<?xml version="1.0"?>
<package format="2">
  <name>unit_5_services</name>
  <version>0.0.0</version>
  <description>The unit_5_services package</description>

  <maintainer email="user@todo.todo">user</maintainer>

  <license>TODO</license>

  <build_depend>iri_wam_reproduce_trajectory</build_depend>
  <build_export_depend>iri_wam_reproduce_trajectory</build_export_depend>
  <exec_depend>iri_wam_reproduce_trajectory</exec_depend>

  <buildtool_depend>catkin</buildtool_depend>
  
  <build_depend>rospy</build_depend>
  <build_export_depend>rospy</build_export_depend>
  <exec_depend>rospy</exec_depend>

  <build_depend>std_msgs</build_depend>  
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>

  <build_depend>message_generation</build_depend>

  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>

  <export>

  </export>
</package>
```

## Create python script for the service server

```
cd ../scripts
touch bb8_move_custom_service_server.py
chmod +x bb8_move_custom_service_server.py
```

Edit the file to add the following code:

```
#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
from unit_5_services.srv import MoveCircleCustom, MoveCircleCustomResponse
from geometry_msgs.msg import Twist

def my_callback(request):

    print("Request Data==> duration="+str(request.duration))

    t_end = rospy.Time.now() + rospy.Duration(request.duration)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    cmd = Twist()
    cmd.linear.x = 0.5
    cmd.angular.z = 0.5

    rate = rospy.Rate(2)

    while rospy.Time.now() < t_end :
        pub.publish(cmd)
        rate.sleep()

    stop_cmd = Twist()
    pub.publish(stop_cmd)

    return MoveCircleCustomResponse(success=True)

rospy.init_node('bb8_move_custom_service_server') 
my_service = rospy.Service('/move_bb8_in_circle_custom', MoveCircleCustom , my_callback) # create the Service called my_service with the defined callback
rospy.spin() # maintain the service open.
```

## Create a launch file for the previous python

```
touch start_bb8_move_custom_service_server.launch
```

<launch>

    <node pkg="unit_5_services" type="bb8_move_custom_service_server.py" name="move_bb8_custom_server" output="screen"/>

</launch>

## Create python script to call the service /move_bb8_in_circle_custom

```
touch call_bb8_move_custom_service_server.py
chmod +x call_bb8_move_custom_service_server.py
```

Edit the file to add the following code:

```
#! /usr/bin/env python

import rospy
import rospkg

from unit_5_services.srv import MoveCircleCustom, MoveCircleCustomRequest

rospy.init_node('move_circle_custom_client')

rospy.wait_for_service('/move_bb8_in_circle_custom')

move_circle_custom_service = rospy.ServiceProxy('/move_bb8_in_circle_custom', MoveCircleCustom)

request = MoveCircleCustomRequest()
request.duration = 10

response = move_circle_custom_service(request)
```

## Create launch file to execute the last .py

```
touch call_bb8_move_custom_service_server.launch

```
<launch>

    <node pkg="unit_5_services" type="call_bb8_move_custom_service_server.py" name="call_bb8_custom_client" output="screen"/>

</launch>
```

# Execute the files

To make it work there are multiple ways but here the server and client are separated so we need two terminals.

In the first one we can either run the first .py or launch its .launch.

Once that is running we can execute the service by calling it ourselves in a second terminal with `rosservice call /move_bb8_in_circle_custom "duration: 10"`

Otherwise, we can also run the second .py or launch the second .launch file in this second terminal.

# **!!! It is important to do both things otherwise nothing happens !!!**