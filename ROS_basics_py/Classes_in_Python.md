# Exemple to move bb8 in a circle using a custom service with duration as input parameter

## Class file

```
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class MoveBB8():
    
    def __init__(self):
        self.bb8_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)
        
    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.bb8_vel_publisher.get_num_connections()
            if connections > 0:
                self.bb8_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def move_bb8(self, duration, linear_speed=0.2, angular_speed=0.2):
        
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        
        rospy.loginfo("Moving BB8!")
        # self.publish_once_in_cmd_vel()

        time = rospy.Time.now()

        while rospy.Time.now() - time < rospy.Duration(duration) :
            self.bb8_vel_publisher.publish(self.cmd)
            self.rate.sleep()

        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.bb8_vel_publisher.publish(self.cmd)
            
# if __name__ == '__main__':
#     rospy.init_node('move_bb8_test', anonymous=True)
#     movebb8_object = MoveBB8()
#     try:
#         movebb8_object.move_bb8()
#     except rospy.ROSInterruptException:
#         pass
```

## Service server file

```
#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse 
from bb8_move_circle_class import MoveBB8
from my_python_class.srv import MoveDuration, MoveDurationResponse


def my_callback(request):
    rospy.loginfo("The Service move_bb8_in_circle has been called")

    movebb8_object = MoveBB8()
    movebb8_object.move_bb8(request.duration)

    rospy.loginfo("Finished service move_bb8_in_circle")
    return MoveDurationResponse(success=True)

if __name__ == "__main__":
    rospy.init_node('service_move_bb8_in_circle_server') 
    my_service = rospy.Service('/move_bb8_in_circle', MoveDuration , my_callback)
    rospy.loginfo("Service /move_bb8_in_circle Ready")
    rospy.spin() # keep the service open.
```

## Call the service

```
rosservice call /move_bb8_in_circle "duration: 10"
```