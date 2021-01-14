#!/usr/bin/env python
import rospy,sys
from gazebo_msgs.msg import ModelStates

def callback(all_model):
    rospy.loginfo(all_model.pose[1].position.x)

def listener():
    
    rospy.init_node("track_ball",anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates,callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
