#!/usr/bin/env python3
# A ROS node to move the robot in a square

import rospy

# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist

# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry

# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion

# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi

class Square():
    def callback_function(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        # convert orientation co-ords to roll, pitch & yaw 
        # (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z
        # so assign these to class variables (so that we can
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z
            # set the second reference position:
            self.x1 = self.x
            self.y1 = self.y
            self.theta_z1 = self.theta_z

    def __init__(self):
        node_name = "move_square"
        # a flag if this node has just been launched
        self.startup = True

        # setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        # variables for a second "reference position":
        self.x1 = 0.0
        self.y1 = 0.0
        self.theta_z1 = 0.0

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

        # state of the robot:
        self.state = 1
        self.move_forward()


    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True

    # method to move the robot forwards:
    def move_forward(self):
        self.vel = Twist()
        # set the linear velocity to 0.2 m/s:
        self.vel.linear.x = 0.2
        # set the angular velocity to zero:
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)

    # method to turn the robot:
    def turn(self):
        self.vel = Twist()
        # set the linear velocity to zero:
        self.vel.linear.x = 0.0
        # set the angular velocity to 0.5 rad/s:
        self.vel.angular.z = 0.5
        self.pub.publish(self.vel)

    # method to stop the robot:
    def stop(self):
        self.vel = Twist()
        self.pub.publish(self.vel)

    def moved_1m(self):
        # calculate the distance moved since the last reference position:
        distance = sqrt(pow((self.x - self.x1), 2) + pow((self.y - self.y1), 2))
        # if the distance moved is greater than 1m, then return True:
        return True if distance > 1.0 else False

    def turned_90deg(self):
        # calculate the angle turned since the last reference position:
        angle = self.theta_z - self.theta_z1
        # if the angle turned is greater than 90 degrees, then return True:
        return True if abs(angle) > pi/2 else False

    def reset_reference_position(self):
        # reset the reference position:
        self.x1 = self.x
        self.y1 = self.y
        self.theta_z1 = self.theta_z

    def calculate_drift_from_start(self):
        # calculate the distance moved since the first reference position:
        sign = 1 if sqrt(pow(self.x0, 2) + pow(self.y0, 2)) - sqrt(pow(self.x, 2) + pow(self.y, 2)) > 0 else -1
        distance = sign * sqrt(pow((self.x - self.x0), 2) + pow((self.y - self.y0), 2))
        # calculate the angle turned since the first reference position:
        angle = self.theta_z - self.theta_z0
        # return the distance and angle:
        return distance, angle

    def print_drift_from_start(self):
        # obtain the distance and angle:
        distance, angle = self.calculate_drift_from_start()
        # print the distance and angle:
        rospy.loginfo(f"distance: {distance}m, angle: {angle}rad")


    def main_loop(self):
        while not self.ctrl_c:
            # here is where your code would go to control the motion of your
            # robot. Add code here to make your robot move in a square of
            # dimensions 1 x 1m...

            if(self.state == 0):
                if(self.turned_90deg()):
                    self.reset_reference_position()
                    self.move_forward()
                    self.state = 1
            elif(self.state == 1):
                if(self.moved_1m()):
                    self.turn()
                    self.state = 0
            else:
                rospy.loginfo("ERROR: Unknown state")



            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == "__main__":
    node = Square()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
