#!/usr/bin/python
import cv2 as cv
import pyzbar.pyzbar as pyzbar
import rospy
from geometry_msgs.msg import Pose2D,PoseStamped

def decodebarcode(image):
    image_gray=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    barcodes=pyzbar.decode(image_gray)
    detected_flag=False
    if barcodes!=[]: 
    #and barcodes[0].type=='CODE128':
        detected_flag=True
        for barcode in barcodes:
            for point in barcode.polygon:
                cv.circle(image,point,2,(0,0,255),8)
            #cv.line(image,point0,point1,(0,0,255),2)
            # point0=barcode.polygon[0]
            # point1=barcode.polygon[1]
            # point2=barcode.polygon[2]
            # point3=barcode.polygon[3]
            # cv.circle(image,point0,2,(0,255,0),8)
            # cv.circle(image,point1,2,(0,255,0),8)
            # cv.circle(image,point2,2,(0,255,0),8)
            # cv.circle(image,point3,2,(0,255,0),8)
            # cv.line(image,point0,point1,(0,0,255),2)
            # cv.line(image,point1,point2,(0,0,255),2)
            # cv.line(image,point2,point3,(0,0,255),2)
            # cv.line(image,point3,point0,(0,0,255),2)
            cv.imshow("cap",image)
            data=barcode.data
    return detected_flag


WINDOW_W=640
WINDOW_H=480
rospy.init_node("find_barcode_node")
barcode_pub=rospy.Publisher('barcode_pixel',Pose2D, queue_size=10)
start_param=False
end_param=False
rospy.set_param('/find_barcode_node/start', False)
rospy.set_param('/find_barcode_node/end',False)
rate = rospy.Rate(10)
while (not start_param)and(not rospy.is_shutdown()):
    start_param=rospy.get_param('/find_barcode_node/start')
    print("waiting to start...")
    rate.sleep()
cap = cv.VideoCapture(0)
barcode_info=Pose2D()
barcode_info.x=WINDOW_W/2
barcode_info.y=WINDOW_H/2
barcode_info.theta=0


while (not end_param)and(not rospy.is_shutdown()):
    end_param=rospy.get_param('/find_barcode_node/end')
    ret, frame = cap.read()
    if not ret:
        print("read_error!!!")
    if(decodebarcode(frame)):
        print("detected!!!")
        barcode_info.theta=1
    else:
        barcode_info.theta=0
        cv.imshow("cap", frame)
    barcode_pub.publish(barcode_info)
    cv.waitKey(1)
    # if cv.waitKey(1) & 0xff == ord('q'):
    #     break
cap.release()
cv.destroyAllWindows()

