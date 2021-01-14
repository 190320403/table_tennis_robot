#!/user/bin/env python
import rospy
from ModelStates.msg import geometry_msgs/Pose[]

def callback(pose):
    rospy.loginfo(pose[1])

def listener():
    
    rospy.init_node("track_ball",anonymous=True)
    rospy.Subscriber("/gazebo/model_states", geometry_msgs/Pose[] ,callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
