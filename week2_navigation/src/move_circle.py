#!/usr/bin/env python3
# A simple ROS publisher node in Python to move the robot in a circle

import rospy 
from geometry_msgs.msg import Twist 

class Publisher(): 

    def __init__(self): 
        self.node_name = "move_circle" 
        topic_name = "cmd_vel" 

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c: 
            move = Twist()
            move.linear.x = 0.2
            move.angular.z = 0.5
            self.pub.publish(move)
            self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = Publisher() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass
