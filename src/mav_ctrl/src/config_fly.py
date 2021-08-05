#!/usr/bin/python
import time
import rospy
from commander import Commander
from geometry_msgs.msg import PoseStamped
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from sensor_msgs.msg import Imu
from pyquaternion import Quaternion

take_off_height=0.6
takeoff_ok=False

def local_pose_callback(msg):#check if take off succed
    distance_tolarance=0.1
    global takeoff_ok
    local_pose=msg
    dx=local_pose.pose.position.x
    dy=local_pose.pose.position.y
    dz=local_pose.pose.position.z-0.6
    ds=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
    #print(ds)
    #print(takeoff_ok)
    if ds<distance_tolarance:
        takeoff_ok=True
    else:
        takeoff_ok=False

def circle(drone,start_yaw=-180,radius=1,points_num=50):
    PI=3.1415926535
    delta_theta=2*PI/points_num
    step=radius*delta_theta
    drone.turn(start_yaw)
    time.sleep(2)
    yaw=start_yaw
    for i in range(points_num):
        yaw=yaw+delta_theta*180/PI
        print(yaw)
	print(step)
        drone.turn(yaw)
        time.sleep(1)
        drone.move(0,-step,0)
        time.sleep(1)

local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_pose_callback)
drone = Commander(1)     #need to not be0
time.sleep(2)
rate = rospy.Rate(10)
while not takeoff_ok:
    print("takeoff not ok!")
    rate.sleep()
print("takeoff ok!")
time.sleep(2)

circle(drone)

drone.land()

