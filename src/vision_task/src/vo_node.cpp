#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Vector3Stamped.h>
#include<geometry_msgs/TwistStamped.h>
#include<mavros_msgs/State.h>
#include<sensor_msgs/Imu.h>
#include"Drv_Ano_of.h"
#include <wiringSerial.h>
#include<wiringPi.h>
#include<iostream>
#include<stdio.h>
#include <math.h>
//#include<Eigen/Eigen>

mavros_msgs::State current_state;
void mavros_state_callback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state=*msg;
}

sensor_msgs::Imu global_imu;
float current_heading=0;
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    global_imu=*msg;
    // float q1=global_imu.orientation.x;
    // float q2=global_imu.orientation.y;
    // float q3=global_imu.orientation.z;
    // float q0=global_imu.orientation.w;
    //current_heading=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}

void confirm_pose(geometry_msgs::PoseStamped& pose,geometry_msgs::PoseStamped base_pose)
{
    pose.pose.position.x=pose.pose.position.x-base_pose.pose.position.x;
    pose.pose.position.y=pose.pose.position.y-base_pose.pose.position.y;
    
}

void FLU2ENU(geometry_msgs::PoseStamped& pose,geometry_msgs::PoseStamped& last_pose,geometry_msgs::PoseStamped& pose_ENU)
{
    float delta_x=pose.pose.position.x-last_pose.pose.position.x;
    float delta_y=pose.pose.position.y-last_pose.pose.position.y;
    pose_ENU.pose.position.x += delta_x*cos(current_heading) - delta_y*sin(current_heading);
    pose_ENU.pose.position.y += delta_x*sin(current_heading) + delta_y*cos(current_heading);
    pose_ENU.pose.position.z= pose.pose.position.z;
    pose_ENU.header.stamp=pose.header.stamp;
    pose_ENU.pose.orientation.w=pose.pose.orientation.w;
    pose_ENU.pose.orientation.x=pose.pose.orientation.x;
    pose_ENU.pose.orientation.y=pose.pose.orientation.y;
    pose_ENU.pose.orientation.z=pose.pose.orientation.z;
}

int main(int argc, char **argv)
{
    ROS_INFO("start  sentting vision position!");
    ros::init(argc, argv, "vo_node");
    ros::NodeHandle nh;
    ros::Publisher  vision_pose_pub= nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
    ros::Publisher  vision_pose_raw_pub= nh.advertise<geometry_msgs::PoseStamped>("vision_pose_raw", 10);

    ros::Subscriber mavros_state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,mavros_state_callback);
    ros::Subscriber imu_sub=nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",10,imu_callback);

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped last_pose;
    geometry_msgs::PoseStamped base_pose;
    geometry_msgs::PoseStamped pose_ENU;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.header.frame_id="base_link";
    last_pose=pose;
    base_pose=pose;
    pose_ENU=pose;
    pose_ENU.header.frame_id="base_link";

    char first_pose_flag=1;

    int fd;
    if(wiringPiSetup() == -1){
            printf("init pin failed\n");
            return 0;
    }
    fd = serialOpen("/dev/ttyAMA0",500000);//初始化串口
    if(fd == -1){
            printf("open serial failed\n");
            return 0;
    }

    ros::Time last_time = ros::Time::now();
    ros::Time now_time=ros::Time::now();
    int mode_flag=0;
    while(ros::ok()){
        if(serialDataAvail(fd)!=-1)
        {
            unsigned char rxstate=0;
            int data=serialGetchar(fd);
            rxstate=AnoOF_GetOneByte(data);
            now_time=ros::Time::now();
            if(rxstate==6)
            {
                float pose_x=ano_of.intergral_x/100.0;//以m为单位
                float pose_y=ano_of.intergral_y/100.0;//以m为单位
                float pose_z=ano_of.of_alt_cm/100.0;//以m为单位

                pose.pose.orientation.w=ano_of.quaternion[0];
                pose.pose.orientation.x=ano_of.quaternion[1];
                pose.pose.orientation.y=ano_of.quaternion[2];
                pose.pose.orientation.z=ano_of.quaternion[3];

                if(pose_z>3)
                    pose_z=0;
                pose.pose.position.x = pose_x;
                pose.pose.position.y =pose_y;
                pose.pose.position.z = pose_z;

                pose.header.stamp=ros::Time::now();

                if(now_time.toSec()-last_time.toSec()>(1/30.0))
                {
                    if(!current_state.armed)
                    {
                        if(mode_flag==1)
                        {
                            mode_flag=0;
                        }
                    }
                    else
                    {
                        if(mode_flag==0)
                        {
                            base_pose=last_pose;
                            mode_flag=1;
                            first_pose_flag=1;
                            pose_ENU.pose.position.x=0;
                            pose_ENU.pose.position.y=0;
                        }
                    }
                    confirm_pose(pose,base_pose);

                    if(first_pose_flag)
                    {
                        last_pose=pose;
                        first_pose_flag=0;
                    }

                    FLU2ENU(pose,last_pose,pose_ENU);
                    float q1_v=pose.pose.orientation.x;
                    float q2_v=pose.pose.orientation.y;
                    float q3_v=pose.pose.orientation.z;
                    float q0_v=pose.pose.orientation.w;
                    float q1_i=global_imu.orientation.x;
                    float q2_i=global_imu.orientation.y;
                    float q3_i=global_imu.orientation.z;
                    float q0_i=global_imu.orientation.w;
                    current_heading=atan2(2*(q0_v*q3_v+q1_v*q2_v),1-2*(q2_v*q2_v+q3_v*q3_v));

                    std::cout<<"[FLU]"<<"("<<pose.pose.position.x<<","<<pose.pose.position.y<<","<<pose.pose.position.z<<")"<<std::endl;
                    std::cout<<"[ENU]"<<"("<<pose_ENU.pose.position.x<<","<<pose_ENU.pose.position.y<<","<<pose_ENU.pose.position.z<<")"<<std::endl;
                    std::cout<<"[heading]"<<current_heading*180/3.1415926<<std::endl;
                    //std::cout<<"[vision RPY]"<<atan2(2*(q0_v*q1_v+q2_v*q3_v),1-2*(q1_v*q1_v+q2_v*q2_v))<<","<<asin(2*(q0_v*q2_v-q1_v*q3_v))<<","<<atan2(2*(q0_v*q3_v+q1_v*q2_v),1-2*(q2_v*q2_v+q3_v*q3_v))<<std::endl;
                    std::cout<<"[vision YAW]"<<atan2(2*(q0_v*q3_v+q1_v*q2_v),1-2*(q2_v*q2_v+q3_v*q3_v))*180/3.14159<<std::endl;
                    std::cout<<"[percent]"<<atan2(2*(q0_v*q3_v+q1_v*q2_v),1-2*(q2_v*q2_v+q3_v*q3_v))*180/3.14159-current_heading*180/3.1415926<<std::endl;             
                    //std::cout<<"[imu RPY]"<<atan2(2*(q0_i*q1_i+q2_i*q3_i),1-2*(q1_i*q1_i+q2_i*q2_i))<<","<<asin(2*(q0_i*q2_i-q1_i*q3_i))<<","<<atan2(2*(q0_i*q3_i+q1_i*q2_i),1-2*(q2_i*q2_i+q3_i*q3_i))<<std::endl;
                    vision_pose_pub.publish(pose_ENU);
                    vision_pose_raw_pub.publish(pose);

                    ros::spinOnce();
                    last_time=now_time;
                    last_pose=pose;
                    //if(std::waitKey(100)==27)break;
                }
            }
        }
    }
    return 0;
}
