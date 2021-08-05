# !usr/bin/env python
from threading import local
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math


class Commander:
    def __init__(self,n=0):
        rospy.init_node("commander_node_"+str(n))
        rate = rospy.Rate(20)
        self.position_target_pub = rospy.Publisher('gi/set_pose/position', PoseStamped, queue_size=10)
        self.yaw_target_pub = rospy.Publisher('gi/set_pose/orientation', Float32, queue_size=10)
        self.custom_activity_pub = rospy.Publisher('gi/set_activity/type', String, queue_size=10)


    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))


    def turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)

    
    # land at current position
    def land(self):
        self.custom_activity_pub.publish(String("LAND"))


    # hover at current position
    def hover(self):
        self.custom_activity_pub.publish(String("HOVER"))


    # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))


    def set_pose(self, x=0, y=0, z=2, BODY_FLU = True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if BODY_FLU:
            pose.header.frame_id = 'base_link'

        else:
            pose.header.frame_id = 'map'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        return pose

    def gene_arc_points(self,r,theta,num,cur_heading,offset,flag=1,direction=0):
        '''
        flag =1: turn left
        flag=-1: turn right
        heading=0: heading to the const direction
        heading=1:heading to the 
        '''
        self.traj=[[0,0]]
        self.rel_traj=[]
        self.local_traj=[]
        theta=theta*3.14/180
        cur_heading=cur_heading*3.14/180
        for i in range(num):
            alpha=theta/num*(i+1)
            self.traj.append([r*math.sin(alpha),flag*(r-r*math.cos(alpha))])
            self.rel_traj.append([self.traj[i+1][0]-self.traj[i][0],self.traj[i+1][1]-self.traj[i][1]])
            self.local_traj.append([offset[0]+self.traj[i+1][0]*math.cos(cur_heading)-self.traj[i+1][1]*math.sin(cur_heading),
                offset[1]+self.traj[i+1][0]*math.sin(cur_heading)+self.traj[i+1][1]*math.cos(cur_heading),offset[2]])

    def follow_arc(self,time_step=0.6):
        for i in self.local_traj:
            self.move(i[0],i[1],i[2],BODY_OFFSET_ENU=False)
            time.sleep(time_step)

if __name__ == "__main__":
    
    con = Commander()
    #rospy.set_param('time_step',1)
    time.sleep(2)
    #con.move(1, 0, 0)
    time.sleep(2)
    con.gene_arc_points(1.5,360,15)
    con.follow_arc()
    con.gene_arc_points(1.5,360,15,-1)
    con.follow_arc()
    time.sleep(2)
    con.land()


