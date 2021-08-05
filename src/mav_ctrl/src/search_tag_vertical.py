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

def distence_sensor_callback(msg):  
    global distence
    distence=msg

def get_rel_pose(apriltag_raw_pose, x_len=640, y_len=480):
    ref_point = [x_len/2, y_len/2]
    apriltag_rel_pose = [-apriltag_raw_pose[1] +
                         ref_point[1], -apriltag_raw_pose[0]+ref_point[0]]
    return apriltag_rel_pose

flag=False
distence=0
apriltag_raw_pose = None
april_sub = rospy.Subscriber(
    '/apriltag_pixel', Pose2D, apriltag_pixel_callback)
# distence_sub=rospy.Subscriber('/distence_sensor',distence_sensor_callback)  # msg type need to be completed
rospy.set_param('/apriltag_pixel/k2', 0.002)
rospy.set_param('/distence_sensor/follow_thershold',1) 
rospy.set_param('/distence_sensor/follow_distence',0.5)
#rospy.set_param('/apriltag_pixel/plot_flag',False)
apriltag_pose_history=[[],[]]
file=open('/home/pi/1.txt','w')
drone = Commander(2)     #need to not be0
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if flag:
        rel_pose = get_rel_pose(apriltag_raw_pose)
        file.write(str(rel_pose[1])+' '+str(rel_pose[0])+'\n')
        k = rospy.get_param('/apriltag_pixel/k2')
        if  distence<=rospy.get_param('/distence_sensor/follow_thershold') and distence>=0.1:
            dist_to_move=distence-rospy.get_param('/distence_sensor/follow_distence')
        else:
            dist_to_move=0
        drone.move(dist_to_move,k*rel_pose[1],k*rel_pose[0])  #changed
        print(k*rel_pose[0],k*rel_pose[1], k)
    else:
        print('not in the vision')
    rate.sleep()
