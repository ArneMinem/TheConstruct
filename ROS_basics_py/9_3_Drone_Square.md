# Make the drone fly in a square with an action server and custom action

```
#! /usr/bin/env python
import rospy

import actionlib

from actionlib_tutorials.msg import FibonacciFeedback, FibonacciResult, FibonacciAction
from move_drone_square_pkg.msg import MoveDroneFeedback, MoveDroneResult, MoveDroneAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class MoveDroneClass(object):

    _feedback = MoveDroneFeedback()
    _result = MoveDroneResult()
        
    def __init__(self):

        self._as = actionlib.SimpleActionServer("move_drone_as", MoveDroneAction, self.goal_callback, False)
        self._as.start()
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    def goal_callback(self, goal):

        t0 = rospy.Time.now()

        r = rospy.Rate(2)
        success = True
        side_time = goal.side_length
        mvs = [[0.5, 0.0], [0.0, 0.5], [-0.5, 0.0], [0.0, -0.5]]
        cmd = Twist()

        takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)

        while True:
                connections = takeoff_pub.get_num_connections()
                if connections > 0:
                    takeoff_pub.publish(Empty())
                    rospy.loginfo("Drone takeoff")
                    break
                else:
                    r.sleep()

        for i in range(4):
            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                self._as.set_preempted()
                success = False
                break

            self._feedback.current_step = f"Moving side {i+1}"
            self._as.publish_feedback(self._feedback)

            cmd.linear.x = mvs[i][0]
            cmd.linear.y = mvs[i][1]

            while rospy.Time.now() < t0 + rospy.Duration((i+1) * side_time):
                self._cmd_pub.publish(cmd)
                r.sleep()
        
        cmd = Twist()
        self._cmd_pub.publish(cmd)

        land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)

        while True:
                connections = land_pub.get_num_connections()
                if connections > 0:
                    land_pub.publish(Empty())
                    rospy.loginfo("Drone landing")
                    break
                else:
                    r.sleep()

        if success:
            duration = rospy.Time.now() - t0
            self._result.message = "Square path completed in: " + str(duration.to_sec()) + " seconds"
            self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('move_drone_square_server')
  MoveDroneClass()
  rospy.spin()
```

```
cmake_minimum_required(VERSION 3.0.2)
project(move_drone_square_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  message_generation
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveDrone.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_drone_square_pkg
 CATKIN_DEPENDS rospy std_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
```

```
<?xml version="1.0"?>
<package format="2">
  <name>move_drone_square_pkg</name>
  <version>0.0.0</version>
  <description>The move_drone_square_pkg package</description>

  <maintainer email="user@todo.todo">user</maintainer>
  
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <build_depend>actionlib_msgs</build_depend>
  <exec_depend>actionlib_msgs</exec_depend>

  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

```
# Goal
int32 side_length   # length of each side of the square (seconds or meters, depending on your design)

---
# Result
bool success
string message

---
# Feedback
string current_step   # e.g. "Moving forward", "Turning left"
```

## Run the action server

`rosrun move_drone_square_pkg move_drone_square_action_server.py`

## Send a goal

In another terminal:
`rostopic pub /move_drone_as/goal move_drone_square_pkg/MoveDroneActionGoal [TAB][TAB]`