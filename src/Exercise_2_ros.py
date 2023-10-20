#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from lab2_robotics import * # Assuming you have these functions
import math


# Global variables for resolved-rate motion control
d = np.array([267, 0, 0, 342.5, 0, 97])
q = np.array([0, -1.3849179, 1.3849179, 0, 0, 0])
alpha = np.array([-math.pi/2, 0, -math.pi/2, math.pi/2, - math.pi/2, 0])
a = np.array([0, 289.48866, 77.5, 0, 76, 0])
revolute = [True, True, True, True, True, True]

# Desired end-effector position
#sigma_d = np.array([-1, 50, -150])
T = kinematics(d, q, a, alpha)
print(T)
#sigma_d = T[-1][0:3,-1]
sigma_d = np.array([700, 0, 200])
print(sigma_d)

# Control gains (you may need to adjust these)
K = np.diag([15, 15, 15])
abc = [0,0,0,0,0,0]

# Callback function to process joint states and control the arm
def joint_states_callback(msg):
    global d, q, a, alpha, revolute, sigma_d, K, abc
    abc = msg.position
    # Extract current joint positions
    current_joint_positions = np.array(msg.position+q)
    #abc = current_joint_positions
    # Update robot
    T = kinematics(d, current_joint_positions, a, alpha)
    J = jacobian(T, revolute)
    J1 = J[0:3,:]
    T = np.array(T)    
    sigma = np.array([T[-1][0][-1], T[-1][1][-1], T[-1][2][-1]])
    err = (sigma_d - sigma).reshape((3, 1)) 
    dq1 = np.linalg.pinv(J1) @ (K @ err)
    dq2 = dq1[:, 0] 
    abc += dt * dq2 
    # Publish joint velocities
    pub = rospy.Publisher('/xarm/xarm6_traj_controller/command', JointTrajectory, queue_size=10)
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    point = JointTrajectoryPoint()
    point.positions = abc.tolist() # Set current joint positions
    #print(dq2.tolist())
    #print(len(dq2))
    #point.velocities = dq2.tolist()  # Set desired joint velocities
    #point.accelerations = []  # Set desired joint accelerations (optional)
    #point.effort = []
    point.time_from_start = rospy.Duration(1)  # Adjust as needed
    rate = rospy.Rate(1/dt)

    msg.points.append(point)
    if math.dist(sigma_d,T[-1][0:3,-1])>10:   
        pub.publish(msg)
        #rate.sleep()
    else:
        print("position reached")
        rospy.signal_shutdown("User requested shutdown")
        # msg = JointTrajectory()
        # msg.header.stamp = rospy.Time.now()
        # msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        # point = JointTrajectoryPoint()
        # point.velocities = [0,0,0,0,0,0]
        # point.time_from_start = rospy.Duration(1) 
        # msg.points.append(point)
        # pub.publish(msg)
        # rospy.signal_shutdown("User requested shutdown")
# Initialize the ROS node
rospy.init_node('control_arm_node')

# Set the rate (adjust as needed)
dt = 0.1


# Subscribe to the joint states topic
rospy.Subscriber('/xarm/joint_states', JointState, joint_states_callback)

# Spin to keep the node alive
rospy.spin()
