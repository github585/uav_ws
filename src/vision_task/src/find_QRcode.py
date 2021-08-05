#!/usr/bin/python
import cv2 as cv
import pyzbar.pyzbar as pyzbar
import rospy
from geometry_msgs.msg import Pose2D,PoseStamped
center_x=320
center_y=240
def decodeQRcode(image):
    global center_x
    global center_y
    image_gray=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    QRcodes=pyzbar.decode(image_gray)
    detected_flag=False
    if QRcodes!=[] and QRcodes[0].type=='QRCODE':
        detected_flag=True
        for QRcode in QRcodes:
            point0=QRcode.polygon[0]
            point1=QRcode.polygon[1]
            point2=QRcode.polygon[2]
            point3=QRcode.polygon[3]
            center_x=(point0[0]+point1[0]+point2[0]+point3[0])/4.0
            center_y=(point0[1]+point1[1]+point2[1]+point3[1])/4.0
            # cv.circle(image,point0,2,(0,255,0),8)###################################
            # cv.circle(image,point1,2,(0,255,0),8)
            # cv.circle(image,point2,2,(0,255,0),8)
            # cv.circle(image,point3,2,(0,255,0),8)
            # cv.line(image,point0,point1,(0,0,255),2)
            # cv.line(image,point1,point2,(0,0,255),2)
            # cv.line(image,point2,point3,(0,0,255),2)
            # cv.line(image,point3,point0,(0,0,255),2)
            # cv.imshow("cap",image)
            data=QRcode.data
    return detected_flag

WINDOW_W=640
WINDOW_H=480
rospy.init_node('find_QRcode_node')
start_param=False
end_param=False
rospy.set_param('/find_QRcode_node/start', False)
rospy.set_param('/find_QRcode_node/end',False)
rate = rospy.Rate(10)
while (not start_param)and(not rospy.is_shutdown()):
    start_param=rospy.get_param('/find_QRcode_node/start')
    print("waiting to start...")
    rate.sleep()

QRcode_pub=rospy.Publisher('QRcode_pixel',Pose2D, queue_size=10)
cap = cv.VideoCapture(0)
QRcode_info=Pose2D()
QRcode_info.x=WINDOW_W/2
QRcode_info.y=WINDOW_H/2
QRcode_info.theta=0


while (not end_param)and(not rospy.is_shutdown()):
    end_param=rospy.get_param('/find_QRcode_node/end')
    ret, frame = cap.read()
    if not ret:
        print("read_error!!!")
    if(decodeQRcode(frame)):
        print("detected!!!")
        QRcode_info.theta=1
	
    else:
        QRcode_info.theta=0
        QRcode_info.x=center_x
        QRcode_info.y=center_y
        # cv.imshow("cap", frame)##################################
    QRcode_pub.publish(QRcode_info)
    cv.waitKey(1)
    # if cv.waitKey(1) & 0xff == ord('q'):
    #     break
cap.release()
cv.destroyAllWindows()

