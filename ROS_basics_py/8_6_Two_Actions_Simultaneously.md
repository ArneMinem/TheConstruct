# Code where a lot is combined

```
#! /usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

"""
class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

"""
# We create some constants with the corresponing vaules from the SimpleGoalState class
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

nImage = 1

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1

# initializes the action client node
rospy.init_node('drone_action_client')

action_server_name = '/ardrone_action_server'

# create the connection to the action server
client = actionlib.SimpleActionClient(action_server_name, ArdroneAction)
# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

# creates a goal to send to the action server
goal = ArdroneGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)

# You can access the SimpleAction Variable "simple_state", that will be 1 if active, and 2 when finished.
#Its a variable, better use a function like get_state.
#state = client.simple_state
# state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
state_result = client.get_state()

rate = rospy.Rate(1)

rospy.loginfo("state_result: "+str(state_result))

takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)

while True:
        connections = takeoff_pub.get_num_connections()
        if connections > 0:
            takeoff_pub.publish(Empty())
            rospy.loginfo("Drone takeoff")
            break
        else:
            rate.sleep()

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

cmd = Twist()
cmd.linear.x = 0.5
cmd.angular.z = 0.2

while state_result < DONE:
    rospy.loginfo("Woving around while waiting for the Server to give a result....")
    pub.publish(cmd)
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))

land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)

while True:
        connections = land_pub.get_num_connections()
        if connections > 0:
            land_pub.publish(Empty())
            rospy.loginfo("Drone landing")
            break
        else:
            rate.sleep()
    
rospy.loginfo("[Result] State: "+str(state_result))
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")
```

- Give values to the goal variable
- Connect to the action server
- Send the goal to the action server (taking pictures for 10 seconds)
- While the action is not finished:
    - Take off the drone
    - Move the drone around
    - Check the state of the action
- When the action is finished, land the drone

- For the takeoff and landing, we wait until there is a connection to the corresponding topic, to be sure that the message will be sent

Here we are publishing to the `/cmd_vel` topic while waiting for the action server to finish its job. But also publishing to '/takeoff' and '/land' topics. So we are using two actions simultaneously.