#include <iostream>
#include<opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>

using namespace cv;
using namespace std;

#define WINDOW_W 640
#define WINDOW_H 480

int main( int argc, char** argv )
{
   Mat imgOriginal,imgHSV,mask;
   int iLowH = 0;
   int iHighH = 179;
   int iLowS = 0; 
   int iHighS = 255;
   int iLowV = 0;
   int iHighV = 46;

   ros::init(argc, argv, "find_pole_node");
   ros::NodeHandle nh;
   ros::Publisher  pole_pixel_pub= nh.advertise<geometry_msgs::Pose2D>("pole_vertical_pixel", 10);

   //这一段很固定，还要记得在主循环加上循环条件和getendparam
   ros::param::set("/find_pole_node/start",false);
   ros::param::set("/find_pole_node/end",false);
   ros::Rate rate(10);
   bool start_param=false;
   bool end_param=false;
   ros::param::get("/find_pole_node/start",start_param);
   ros::param::get("/find_pole_node/end",end_param);
   while(!start_param && ros::ok())
   {
      ros::param::get("/find_blob_node/start",start_param);
      std::cout<<"waiting to start..."<<std::endl;
      rate.sleep();
   }
   ROS_INFO("start  sentting pole pixel!");


   VideoCapture cap("/dev/my_usb_video"); //capture the video from web cam
   if ( !cap.isOpened() )  // if not success, exit program
   {
      cout << "Cannot open the web cam" << endl;
      return -1;
   }
    geometry_msgs::Pose2D center_pixel;
    center_pixel.x=WINDOW_W/2;
    center_pixel.y=0;
    center_pixel.theta=0;
   while (ros::ok()&&!end_param)
   {
      ros::param::get("/find_pole_node/end",end_param);
      bool bSuccess = cap.read(imgOriginal); // read a new frame from video
      if (!bSuccess) //if not success, break loop
      {
         cout << "Cannot read a frame from video stream" << endl;
         break;
      }
      //medianBlur(imgOriginal, imgOriginal,7);
      cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
      inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), mask); //Threshold the image
      //开操作 (去除一些噪点)
      Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
       morphologyEx(mask, mask, MORPH_OPEN, element);
      //闭操作 (连接一些连通域)
      morphologyEx(mask, mask, MORPH_CLOSE, element);
      vector<Vec2f> lines;
      HoughLines(mask,lines,1,CV_PI/180,150,0,0);
      center_pixel.y=0;
      for(size_t i=0;i<lines.size()&&i<10;i++)
      {
         float rho=lines[i][0],theta=lines[i][1];
         if (abs(theta*180/3.14159)<5)
         {
            Point pt1,pt2;
            double a=cos(theta),b=sin(theta);
            double x0=a*rho,y0=b*rho;
            pt1.x=cvRound(x0+1000*(-b));
            pt1.y=cvRound(y0+1000*(a));
            pt2.x=cvRound(x0-1000*(-b));
            pt2.y=cvRound(y0-1000*(a));
            line(imgOriginal,pt1,pt2,Scalar(255,0,0),2,LINE_AA);
            center_pixel.x=x0;
            center_pixel.y=1;
            center_pixel.theta=theta;
            std::cout<<"[find]"<<center_pixel.x<<"  "<<center_pixel.y<<std::endl;
            break;
         }
        pole_pixel_pub.publish(center_pixel);
      }
      imshow("find_lines",imgOriginal);
      imshow("mask",mask);
      waitKey(1);
   }
   return 0;
}