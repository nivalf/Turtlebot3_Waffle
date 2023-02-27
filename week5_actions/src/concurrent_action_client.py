#!/usr/bin/env python3

import rospy 
import actionlib 

from tuos_ros_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback 

node_name = "camera_sweep_action_client"
action_server_name = "/camera_sweep_action_server"

captured_images = 0
def feedback_callback(feedback_data: CameraSweepFeedback): 
    global captured_images
    captured_images = feedback_data.current_image
    print(f"FEEDBACK: Current yaw: {feedback_data.current_angle:.1f} degrees. "
          f"Image(s) captured so far: {captured_images}...")

rospy.init_node(node_name) 

client = actionlib.SimpleActionClient(action_server_name, 
            CameraSweepAction) 
client.wait_for_server() 

goal = CameraSweepGoal()
goal.sweep_angle = 90
goal.image_count = 4

client.send_goal(goal, feedback_cb=feedback_callback) 

# Concurrently do other things while we wait for the action to complete
rate = rospy.Rate(1)
i = 1
print("While we're waiting, let's do our seven-times tables...")

# Client States:
# PENDING=0
# ACTIVE=1
# PREEMPTED=2
# SUCCEEDED=3
# ABORTED=4
# REJECTED=5
# PREEMPTING=6
# RECALLING=7
# RECALLED=8
# LOST=9

while client.get_state() < 2:
    print(f"STATE: Current state code is {client.get_state()}")
    print(f"TIMES TABLES: {i} times 7 is {i*7}")
    i += 1
    rate.sleep()


print(f"RESULT: Action State = {client.get_state()}") 
print(f"RESULT: {captured_images} images saved to {client.get_result()}")
