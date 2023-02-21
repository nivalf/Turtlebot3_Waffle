#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tuos_ros_msgs.srv import Approach, ApproachResponse
from math import isinf

class Approach_Object_Server(): 

    def service_callback(self, service_request):

        service_response = ApproachResponse()

        approach_distance = service_request.approach_distance
        approach_velocity = service_request.approach_velocity

        rospy.loginfo(f"The '{rospy.get_name()}' obtained the request: \n\tapproach_distance: '{approach_distance:.2f} m'\n\tapproach_velocity: '{approach_velocity} m/s'")

        # Handle invalid requests
        if approach_distance < 0:
            service_response.response_message = f'Invalid approach distance {approach_distance:.2f}m. Please choose a distance between 0.12m and 3.5m.'
            return service_response
        
        if approach_distance < 0.12:
            service_response.response_message = f'The laser range finder has a minimum range of 0.12m. Please choose a distance between 0.12m and 3.5m.'
            return service_response
        
        if approach_distance >= 3.5:
            service_response.response_message = f'The laser range finder has a maximum range of 3.5m. Please choose a distance between 0.12m and 3.5m.'
            return service_response
        
        if approach_velocity < 0:
            service_response.response_message = f'Invalid approach velocity {approach_velocity:.2f}m/s.'
            return service_response
        
        if approach_velocity > 0.26:
            service_response.response_message = f'The maximum linear velocity of the robot is 0.26m/s. Please choose a smaller approach velocity.'
            return service_response
        
        # Handle valid requests
        if (isinf(self.range_ahead)):
            service_response.response_message = 'No object detected.'
            return service_response
        elif (self.range_ahead < approach_distance):
            service_response.response_message = f'The robot is already within {approach_distance:.2f}m of the object.'
            return service_response
        elif (self.range_ahead > approach_distance):
            rospy.loginfo(f"The robot is approaching the object...")

            vel = Twist()
            vel.linear.x = approach_velocity
            self.pub.publish(vel)
            
            # Move the robot forward until the object is within the desired distance
            while (self.range_ahead > approach_distance):
                self.rate.sleep()

            # Stop the robot
            vel = Twist()
            self.pub.publish(vel)

            rospy.loginfo(f"The robot has reached the set distance to the object. has stopped moving.")

        service_response.response_message = 'Request complete.'
        return service_response

    # Store the range infront of the robot in a class variable
    def scan_callback(self, scan_data: LaserScan): 
        self.range_ahead = scan_data.ranges[0]

    def __init__(self): 
        service_name = "approach_object_service"

        rospy.init_node(f"{service_name}_server") 
        self.rate = rospy.Rate(10)  # hz

        self.service = rospy.Service(service_name, Approach, self.service_callback) 

        self.sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 

    def main_loop(self):
        rospy.spin() 

if __name__ == '__main__': 
    server_instance = Approach_Object_Server()
    server_instance.main_loop()
