#include <iostream>
#include<opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

using namespace cv;
using namespace std;

#define WINDOW_W 640
#define WINDOW_H 480

void white_balance(Mat& imageSource,Mat& imageTaget)
{
   vector<Mat> imageRGB;
   split(imageSource, imageRGB);
   double R, G, B;
   B = mean(imageRGB[0])[0];
   G = mean(imageRGB[1])[0];
   R = mean(imageRGB[2])[0];
   double KR, KG, KB;
   KB = (R + G + B) / (3 * B);
   KG = (R + G + B) / (3 * G);
   KR = (R + G + B) / (3 * R);
   imageRGB[0] = imageRGB[0] * KB;
   imageRGB[1] = imageRGB[1] * KG;
   imageRGB[2] = imageRGB[2] * KR;
   merge(imageRGB, imageTaget);
}

int main( int argc, char** argv )
 {
   ros::init(argc, argv, "find_blob_node");
   ros::NodeHandle nh;
   ros::Publisher  blob_pixel_pub= nh.advertise<geometry_msgs::Pose2D>("blob_pixel", 10);
   
   ros::param::set("/find_blob_node/start",false);
   ros::param::set("/find_blob_node/end",false);
   ros::Rate rate(20);
   bool start_param=false;
   bool end_param=false;
   ros::param::get("/find_blob_node/start",start_param);
   ros::param::get("/find_blob_node/end",end_param);
   while(!start_param && ros::ok())
   {
      ros::param::get("/find_blob_node/start",start_param);
      std::cout<<"waiting to start..."<<std::endl;
      rate.sleep();
   }
   ROS_INFO("start  sentting blob pixel!");
   VideoCapture cap("/dev/v4l/by-id/usb-Etron_Technology__Inc._USB2.0_Camera-video-index0"); 
    if ( !cap.isOpened()) 
   {
      cout << "Cannot open the web cam" << endl;
      return -1;
   }

   geometry_msgs::Pose2D center_pixel;
   center_pixel.x=WINDOW_W/2;
   center_pixel.y=WINDOW_H/2;
   center_pixel.theta=0;

   int iLowH = 168;
   int iHighH = 179;
   int iLowS = 105; 
   int iHighS = 255;
   int iLowV = 29;
   int iHighV = 255;

   while (ros::ok()&&!end_param)
   {
      ros::param::get("/find_blob_node/end",end_param);
      Mat imgOriginal;
      bool bSuccess = cap.read(imgOriginal); // read a new frame from video
      if (!bSuccess) //if not success, break loop
      {
         cout << "Cannot read a frame from video stream" << endl;
         break;
      }
      white_balance(imgOriginal,imgOriginal);
      //blur(imgOriginal,imgOriginal,Size(7,7));
      //medianBlur(imgOriginal, imgOriginal,7);
      Mat imgHSV;
      vector<Mat> hsvSplit;
      cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
      Mat imgThresholded;
      inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      //开操作 (去除一些噪点)
      Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
      morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
      //闭操作 (连接一些连通域)
      morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;//hierarchy[i][0]:下一个
      findContours(imgThresholded,contours,hierarchy,RETR_CCOMP,CHAIN_APPROX_SIMPLE);
      center_pixel.theta=0;
      if(!hierarchy.empty())
      {
         Point2f center;
         float radius;
         int index=0;
         //Scalar color(rand()&255,rand()&255,rand()&255);
         //drawContours(imgOriginal,contours,0,color,2,8,hierarchy);//index为-1时画所有轮廓
         minEnclosingCircle(contours[0],center,radius);
         //circle(imgOriginal,center,radius, Scalar(255,0,0),3);
        cout<<center.x<<","<<center.y<<endl;
        center_pixel.x=center.x;
        center_pixel.y=center.y;
        center_pixel.theta=1;
      }
      blob_pixel_pub.publish(center_pixel);
      //imshow("Thresholded Image", imgThresholded); //show the thresholded image
      //imshow("Original", imgOriginal); //show the original image
   }
   return 0;
}
