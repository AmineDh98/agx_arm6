#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf 
from tf.transformations import quaternion_from_euler  
import transforms3d.axangles as t_ax    
from dh import *

class forward_kin:
    def __init__():
        self.d = [267, 0, 0, 342.5, 0,97]
        self.theta = [0, -1.3849179, 1.3849179, 0, 0, 0]
        self.alpha = [-math.pi/2, 0, -math.pi/2, math.pi/2, - math.pi/2, 0]
        self.a = [0, 289.48866, 77.5, 0, 76, 0]
        self.arm6 = dh_par(self.d,self.theta,self.alpha,self.a)
        
        
    def forward():
        T = self.arm6.kinematics()[-1]
        J = self.arm6.jacobian(T)
        
        
        
        

    
        
    
    
        