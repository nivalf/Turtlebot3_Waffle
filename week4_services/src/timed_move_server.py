#!/usr/bin/env python3 

# ref: https://lincaolab.github.io/acs6121/labs/la1/week4/move_server/

import rospy
from geometry_msgs.msg import Twist 
from tuos_ros_msgs.srv import TimedMovement, TimedMovementResponse

service_name = "timed_move_service"

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

def callback_function(service_request): 

    vel = Twist()

    service_response = TimedMovementResponse() 

    movement_request = service_request.movement_request
    duration = service_request.duration


    if duration > 0: 

        if(movement_request == 'fwd'):
            print(f"The '{service_name}' Server received a 'fwd' request and the robot will now move for {duration} seconds...") 
            vel.linear.x = 0.1
        elif(movement_request == 'bwd'):
            print(f"The '{service_name}' Server received a 'bwd' request and the robot will now move for {duration} seconds...") 
            vel.linear.x = -0.1
        elif(movement_request == 'left'):
            print(f"The '{service_name}' Server received a 'left' request and the robot will now turn for {duration} seconds...") 
            vel.angular.z = 0.1
        elif(movement_request == 'right'):
            print(f"The '{service_name}' Server received a 'right' request and the robot will now turn for {duration} seconds...") 
            vel.angular.z = -0.1
        else:
            print(f"The '{service_name}' Server received an invalid request and the robot will not move...")
            service_response.success = False
            return service_response

        # Publish the velocity command to /cmd_vel
        pub.publish(vel)

        # Wait for the requested duration
        StartTime = rospy.get_rostime()
        while (rospy.get_rostime().secs - StartTime.secs) < duration:
            continue

        # Stop the robot
        rospy.loginfo(f"{duration} seconds have elapsed, stopping the robot...")
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        pub.publish(vel)

        service_response.success = True
        return service_response

    else: 
        print(f"The '{service_name}' Server received a request with 0 duration and the robot will not move...")
        service_response.success = False
    return service_response

rospy.init_node(f"{service_name}_server") 
my_service = rospy.Service(service_name, TimedMovement, callback_function) 
rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 
rospy.spin() 
