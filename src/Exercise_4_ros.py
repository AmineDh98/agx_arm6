﻿import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lab4_robotics import * # Assuming you have the required modules
import numpy as np
from random import uniform

# Robot model - 6-link manipulator
d = np.array([267, 0, 0, 342.5, 0, 97])
q = np.array([0, -1.3849179, 1.3849179, 0, 0, 0])
alpha = np.array([-math.pi/2, 0, -math.pi/2, math.pi/2, - math.pi/2, 0])
a = np.array([0, 289.48866, 77.5, 0, 76, 0])
        
revolute = [True, True, True, True, True, True]
sigma_d = np.array([600, 0, 700])
robot = Manipulator(d, q, a, alpha, revolute)

# Task hierarchy definition
tasks = [ 
    Position2D("End-effector position", sigma_d.reshape(3,1), 6),
    Orientation2D("End-effector orientation", np.array([0,0,0]).reshape(3,1), 4)
] 

# Simulation params
dt = 1

# Number of tasks
l = len(tasks)
abc = [0,0,0,0,0,0]

# Initialize ROS node
rospy.init_node('xarm_control_node', anonymous=True)

# Callback function for joint states
def joint_states_callback(data):
    global tasks
    global robot
    global d, q, a, alpha, revolute, sigma_d, abc
    #if abc[1] == 0:
    abc = data.position[1:7]
    # Assuming the joint states are in the same order as the DH parameters
    current_joint_positions = np.array(data.position[1:7] )
    robot.update_T(current_joint_positions)
    
    dof = robot.getDOF()

    # Recursive Task-Priority algorithm
    P = np.eye(dof)
    dq = np.zeros((dof, 1))

    for i in range(len(tasks)):
        tasks[i].update(robot)
        Jbar = tasks[i].getJacobian() @ P
        dq = dq + DLS(Jbar, 0.5) @ (tasks[i].getError() - (tasks[i].getJacobian() @ dq))
        P = P - np.linalg.pinv(Jbar) @ Jbar
    
    # Update robot
   
    abc += dt*dq.flatten()
  
    # Publish joint velocities
    vel_msg = JointTrajectory()
    vel_msg.header.stamp = rospy.Time.now()
    vel_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    point = JointTrajectoryPoint()
    point.positions = abc.tolist()
    point.time_from_start = rospy.Duration(1)
    rate = rospy.Rate(1/dt)
    vel_msg.points.append(point)
    T = robot.getEETransform()
    if math.dist(sigma_d,T[0:3,-1])>10:   
        pub.publish(vel_msg)
        #rate.sleep()
    else:
      
        print("position reached")
        rospy.signal_shutdown("User requested shutdown")

# Subscriber for joint states
rospy.Subscriber('/xarm/joint_states', JointState, joint_states_callback)

# Publisher for joint velocities
pub = rospy.Publisher('/xarm/xarm6_traj_controller/command', JointTrajectory, queue_size=10)

# Keep the node running
rospy.spin()
