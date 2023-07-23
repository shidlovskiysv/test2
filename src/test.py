import math 
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#!/usr/bin/env python
import rospy

class ROSNode:
    def __init__(self):
        rospy.init_node("name_node")
        rospy.loginfo("Starting ROSNode as name_node.")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("scan", LaserScan, self.cc)
       # self.rate = rospy.Rate(10) #10Hz
        self.dist=0
        self.err_prev=0
        self.sum=0

    def cc(self,data):
        self.dist=min(data.ranges)
        print(self.dist)
        self.control(0.3,1,150,1/10)

    def control(self, ref, Kp, Kd,Ki):
        self.err_cur=ref-self.dist
        u_p=self.err_cur*Kp
        u_d=(self.err_cur-self.err_prev)*(1)*Kd # 0.1 -> 10 Hz
       
        err_int=(self.sum+(self.err_cur+self.err_prev)*(1/8))
        u_i=(err_int*0.5)*Ki

        cmd_vel = Twist()
        # --- PD -control =====
        cmd_vel.linear.x=0.08
        cmd_vel.angular.z=u_p-u_d#+u_i
        self.pub.publish(cmd_vel)
        #--- mem ----
        self.err_prev=self.err_cur
        self.sum=err_int





if __name__ == "__main__":
    name_node = ROSNode()
    rospy.spin()

