from sys import dont_write_bytecode
from threading import local
import commander
import rospy
from sensor_msgs.msg import Imu
from pyquaternion import Quaternion
from geometry_msgs.msg import PoseStamped

def imu_callback(msg):
        global current_heading
        imu = msg
        current_heading = q2yaw(imu.orientation)
def q2yaw( q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]
        return rotate_z_rad
def local_pose_callback(msg):
    global local_pose
    local_pose = msg

current_heading=0
local_pose=None
local_pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, local_pose_callback)
imu_sub = rospy.Subscriber(
            "/mavros/imu/data", Imu, imu_callback)
drone=commander.Commander(3)
if local_pose != None:
    offset=[local_pose.pose.position.x,local_pose.pose.position.y,local_pose.pose.position.z]
    drone.gene_arc_points(1,360,20,current_heading,offset)
    drone.follow_arc(0.5)
    rospy.sleep(2)
    #offset=[local_pose.pose.position.x,local_pose.pose.position.y,local_pose.pose.position.z]
    #drone.gene_arc_points(1,180,10,current_heading,offset,flag=-1)
    #drone.follow_arc(1)
    drone.land()
