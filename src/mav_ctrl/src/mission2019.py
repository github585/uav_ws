#!/usr/bin/python
import time
import rospy
from commander import Commander
from geometry_msgs.msg import PoseStamped,Pose2D
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from pan_tilt_tools import pan_tilt
from vision_tools import pixel2move

take_off_height=0.6
takeoff_ok=False
find_blob_ok=False
find_blob_time=0
drone = Commander(1)     #need to not be0
find_QRcode_ok=False

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

def find_blob_callback(msg):
    blob=msg
    global find_blob_ok
    global drone
    global find_blob_time
    if blob.theta==1:
        find_blob_ok=True
        if find_blob_time==1:
            m=pixel2move(blob.x,blob.y,2,0.6)
            drone.move(m[0],0,m[2])
        elif find_blob_time==2:
            pass

def find_QRcode_callback(msg):
    QRcode=msg
    global find_QRcode_ok
    if QRcode.theta==1:
        m=pixel2move(QRcode.x,QRcode.y,2,0.6)
        drone.move(m[0],0,m[2])


local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_pose_callback)
blob_sub=rospy.Subscriber("/blob_pixel", Pose2D,find_blob_callback)
QRcode_sub=rospy.Subscriber("/QRcode_pixel", Pose2D,find_QRcode_callback)

time.sleep(2)
rate = rospy.Rate(10)
while not takeoff_ok:
    print("takeoff not ok!")
    rate.sleep()
print("takeoff ok!")
time.sleep(2)

rospy.set_param('/find_blob_node/start', True)
for i in range(15):
    drone.move(0.2,0,0)
    time.sleep(1)
    if find_blob_ok:
        find_blob_time=1
        #alarm
        time.sleep(4)
        rospy.set_param('/find_blob_node/end', True)
        time.sleep(1)
        find_blob_ok=False
        rospy.set_param('/find_QRcode_node/start',True)
    elif find_QRcode_ok:
        time.sleep(4)
        rospy.set_param('/find_QRcode_node/end',True)
        break

drone.move(0.6,0,0)
time.sleep(4)
drone.move(0,1.2,0)
time.sleep(4)
drone.move(-4.2,0,0)
drone.land()
