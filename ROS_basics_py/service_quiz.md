# Problem with the service quiz

My code :

```
#! /usr/bin/env python

import rospy

from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist

def my_callback(request):

    print("Request Data==> side="+str(request.side))
    print("Request Data==> repetitions="+str(request.repetitions))

    s = request.side
    duration = 2*s
    
    r = request.repetitions

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    cmd = Twist()

    rate = rospy.Rate(2)

    for i in range(r):

        # First side
    
        t_end = rospy.Time.now() + rospy.Duration(duration)
        cmd.linear.x = 1.0
        cmd.linear.y = 0.0

        while rospy.Time.now() < t_end :
            print(1)
            pub.publish(cmd)
            rate.sleep()
            print(2)

        # Second side

        stop_cmd = Twist()
        pub.publish(stop_cmd)
        t_end = rospy.Time.now() + rospy.Duration(duration)
        cmd.linear.x = 0.0
        cmd.linear.y = 1.0

        while rospy.Time.now() < t_end :
            print(3)
            pub.publish(cmd)
            rate.sleep()
            print(4)

        # Third side

        stop_cmd = Twist()
        pub.publish(stop_cmd)
        t_end = rospy.Time.now() + rospy.Duration(duration)
        cmd.linear.x = -1.0
        cmd.linear.y = 0.0
        while rospy.Time.now() < t_end :
            print(5)
            cmd.linear.x = -1.0
            cmd.linear.x = 0.0
            pub.publish(cmd)
            rate.sleep()
            print(6)

        # Forth side

        stop_cmd = Twist()
        pub.publish(stop_cmd)
        t_end = rospy.Time.now() + rospy.Duration(duration)
        cmd.linear.x = 0.0
        cmd.linear.y = -1.0

        while rospy.Time.now() < t_end :
            print(7)
            pub.publish(cmd)
            rate.sleep()
            print(8)

    stop_cmd = Twist()
    pub.publish(stop_cmd)

    return BB8CustomServiceMessageResponse(success=True)

rospy.init_node('move_bb8_in_square_custom_service_server') 
my_service = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage , my_callback) # create the Service called my_service with the defined callback
rospy.spin() # maintain the service open.
```

**It doesn't work because bb8 can't move in y direction.**

**!!! This should be specified in the quiz !!!**

**!!! At is also impossible to get right angles with the robot !!!**


# Careful for the indications !

In the quiz it is written :

**Create a new service client that calls the service /move_bb8_in_square_custom, and makes BB-8 move in a small square twice and in a bigger square once. It will be called bb8_move_custom_service_client.py. The launch that starts it will be called call_bb8_move_in_square_custom_service_server.launch.**