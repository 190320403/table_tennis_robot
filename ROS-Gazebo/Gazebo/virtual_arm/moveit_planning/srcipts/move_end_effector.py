#!/usr/bin/env python
import rospy,sys
import math
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
"""def callback(ball_pose->pose->position):
    rospy.loginfo(ball_pose.pose)
#    global posex
#    global posez
    
    target_pose = PoseStamped ()
    target_pose.header.frame_id =reference_frame
    target_pose.header.stamp =rospy.Time.now()
    
#    target_pose.pose.position.x = posex
#    target_pose.pose.position.x = all_model.pose[1].position.x
#    target_pose.pose.position.y = -0.000146469
#    target_pose.pose.position.z = all_model.pose[1].position.z
#    target_pose.pose.position.z =posez
    target_pose.pose.orientation.x =0.000338753
    target_pose.pose.orientation.y =0.452045
    target_pose.pose.orientation.z = -0.00346716
    target_pose.pose.orientation.w = 0.891988

    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose,end_effector_link)

    traj = arm.plan()

    arm.execute(traj)
#    rospy.sleep(0.7)
#    arm.go()
#    posex=posex+0.003
#    posez=posez+0.001
#    rospy.loginfo(posex)
#    rospy.loginfo(posez)
	

'''class MoveItIkDemo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)     
        rospy.init_node('move_end_node')                    

        arm = moveit_commander.MoveGroupCommander('arm')

        end_effector_link = arm.get_end_effector_link()

	

	arm.set_pose_reference_frame(reference_frame)

	arm.allow_replanning(True)

	arm.set_goal_position_tolerance(0.1)

	arm.set_named_target('initial_pose')

	arm.go()

	rospy.sleep(0.05)
	rospy.Subscriber("/gazebo/model_states", ModelStates,callback)
        rospy.spin()
	

	
	
if __name__ =="__main__":
    reference_frame = 'base_link'
    MoveItIkDemo()
	
'''
"""



if __name__ =="__main__":

	moveit_commander.roscpp_initialize(sys.argv)     
	rospy.init_node('move_end_node')                    

	arm = moveit_commander.MoveGroupCommander('all_parts')

	end_effector_link = arm.get_end_effector_link()

	reference_frame = 'world'
	arm.set_pose_reference_frame(reference_frame)

	arm.allow_replanning(True)

	arm.set_goal_position_tolerance(0.3)
	arm.set_goal_orientation_tolerance(0.05)
	arm.set_named_target('initial_pose')


	arm.go()
	rospy.sleep(0.02)
	arm.set_named_target('pose_1')


	arm.go()
	rospy.sleep(0.02)
	arm.set_named_target('pose_2')

	arm.go()
	rospy.sleep(0.02)
	arm.set_named_target('pose_3')

	arm.go()
	rospy.sleep(0.02)
	arm.set_named_target('pose_4')

	arm.go()
	rospy.sleep(0.02)
	arm.set_named_target('pose_5')

	arm.go()
	rospy.sleep(0.02)
	arm.set_named_target('pose_6')

	arm.go()
	rospy.sleep(0.02)
	arm.set_named_target('hit_pose')

	arm.go()
	rospy.sleep(0.1)
	arm.set_planning_time(0.2)


	#    posex = -0.1

	#    posez = 0.22
	#   rospy.sleep(0.05)
	#    rospy.Subscriber("ball_pose",Odometry,callback)
	rospy.spin()

























