#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal


def joint_publisher():
    rospy.init_node('joint_trajectory_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    pub = rospy.Publisher('/xarm/xarm6_traj_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    point = JointTrajectoryPoint()
    point.positions = [0, 0, 0, 0, 0, 0]
    point.velocities = [0, 1, 1, 0, 0, 0]
    point.accelerations = [0, 1, 0, 0, 0, 0]
    point.effort = [1, 1, 1, 1, 1, 1]
    point.time_from_start = rospy.Duration(5)  # 1 second

    msg.points.append(point)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
    
    
def publish_joint_trajectory():
    pub_goal = rospy.Publisher('/xarm/xarm6_traj_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    rospy.init_node('joint_trajectory_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        joint_trajectory_goal = FollowJointTrajectoryActionGoal()
        
        # Populate the FollowJointTrajectoryActionGoal message
        # with the relevant data.
        # Example:
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.0, 2.0, 0.0, 0.0, 0.0]  # Set your desired joint positions
        point.time_from_start = rospy.Duration(5.0)  # Set the duration for the trajectory
        joint_trajectory.points.append(point)

        joint_trajectory_goal.goal.trajectory = joint_trajectory
        
        pub_goal.publish(joint_trajectory_goal)
        rate.sleep()
   

if __name__ == '__main__':   
    try:      
        joint_publisher()
    except rospy.ROSInterruptException:
        pass
