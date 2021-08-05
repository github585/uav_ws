#!/usr/bin/python
import time
import RPi.GPIO as GPIO
from PCA9685 import PCA9685

# pwm = PCA9685()
# try:
#     print ("This is an PCA9685 routine")    
#     pwm.setPWMFreq(50)
#     #pwm.setServoPulse(1,500) 
#     pwm.setRotationAngle(1, 0)
    
#     while True:
#         # setServoPulse(2,2500)
#         # for i in range(10,170,1): 
#         #     pwm.setRotationAngle(0, i)   
#         #     if(i<80):
#         #         pwm.setRotationAngle(1, i)   
#         #     time.sleep(0.01)

#         # for i in range(170,10,-1): 
#         #     pwm.setRotationAngle(0, i)   
#         #     if(i<80):
#         #         pwm.setRotationAngle(1, i)            
#         #     time.sleep(0.01)
#         #angle of i2c0 must short than 30 degree 
#         for i in range(4):
#             pwm.setRotationAngle(0,10*i)
#             time.sleep(1)

# except:
#     pwm.exit_PCA9685()
#     print "\nProgram end"
#     exit()

class pan_tilt:
    def __init__(self,yaw_angle=90,pitch_angle=0,sleep_time=0.02):
        self.pwm = PCA9685()
        self.pwm.setPWMFreq(50)
        self.pwm.setRotationAngle(1,yaw_angle)
        self.pwm.setRotationAngle(0,pitch_angle)
    def turn(self,yaw_angle=90,pitch_angle=0):
        self.pwm.setRotationAngle(1,yaw_angle)
        self.pwm.setRotationAngle(0,pitch_angle)
    def scan(self,yaw_angle_start=0,yaw_angle_end=180):#one turn
        self.pwm.setRotationAngle(1,yaw_angle_start)
        for i in range(yaw_angle_start,yaw_angle_end,1):
            self.pwm.setRotationAngle(1,i)
            time.sleep(0.02)
