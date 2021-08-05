#!/usr/bin/python
import time
import rospy
from commander import Commander
from geometry_msgs.msg import Pose2D,PoseStamped
#from matplotlib import pyplot as plt


def apriltag_pixel_callback(msg):
    global apriltag_raw_pose, flag
    apriltag_raw_pose = [msg.x, msg.y]
    flag = msg.theta

def local_pose_callback(msg):
    global local_pose
    local_pose = msg

def get_rel_pose(apriltag_raw_pose, x_len=640, y_len=480):
    ref_point = [x_len/2, y_len/2]
    apriltag_rel_pose = [-apriltag_raw_pose[1] +
                         ref_point[1], -apriltag_raw_pose[0]+ref_point[0]]
    return apriltag_rel_pose

flag=False
apriltag_raw_pose = None
april_sub = rospy.Subscriber(
    '/apriltag_pixel', Pose2D, apriltag_pixel_callback)
local_pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, local_pose_callback)
rospy.set_param('/apriltag_pixel/k', 0.002)
#rospy.set_param('/apriltag_pixel/plot_flag',False)
apriltag_pose_history=[[],[]]
file=open('/home/pi/1.txt','w')
drone = Commander(1)     #need to not be0
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if flag:
        rel_pose = get_rel_pose(apriltag_raw_pose)
        file.write(str(rel_pose[0])+' '+str(rel_pose[1])+'\n')
        k = rospy.get_param('/apriltag_pixel/k')
        height=local_pose.pose.position.z
        drone.move(height*k*rel_pose[0], height*k*rel_pose[1], 0)
        print(height*k*rel_pose[0],height*k*rel_pose[1], k)
    else:
        print('not in the vision')
    rate.sleep()
