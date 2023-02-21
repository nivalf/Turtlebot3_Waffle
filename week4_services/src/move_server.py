#!/usr/bin/env python3 

# ref: https://lincaolab.github.io/acs6121/labs/la1/week4/move_server/

import rospy
from geometry_msgs.msg import Twist 
from tuos_ros_msgs.srv import SetBool, SetBoolResponse 

service_name = "move_service"

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

def callback_function(service_request): 

    vel = Twist()

    service_response = SetBoolResponse() 

    if service_request.request_signal == True: 
        print(f"The '{service_name}' Server received a 'true' request and the robot will now move for 5 seconds...") 

        StartTime = rospy.get_rostime() 

        vel.linear.x = 0.1
        pub.publish(vel) 

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while (rospy.get_rostime().secs - StartTime.secs) < 5: 
            continue

        rospy.loginfo('5 seconds have elapsed, stopping the robot...')

        vel.linear.x = 0.0
        pub.publish(vel) 

        service_response.response_signal = True 
        service_response.response_message = "Request complete."
    else: 
        service_response.response_signal = False
        service_response.response_message = "Nothing happened, set request_signal to 'true' next time."
    return service_response

rospy.init_node(f"{service_name}_server") 
my_service = rospy.Service(service_name, SetBool, callback_function) 
rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 
rospy.spin() 
