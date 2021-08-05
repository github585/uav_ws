#include <iostream>
#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include<iostream>
#include<stdio.h>
#include <geometry_msgs/Pose2D.h>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "apriltag_pose.h" 
}

using namespace std;
using namespace cv;

#define WINDOW_W 640
#define WINDOW_H 480

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "apriltag_pixel_node");
    ros::NodeHandle nh;
    ros::Publisher  apriltag_pixel_pub= nh.advertise<geometry_msgs::Pose2D>("apriltag_pixel", 10);
    geometry_msgs::Pose2D center_pixel;
    center_pixel.x=WINDOW_W/2;
    center_pixel.y=WINDOW_H/2;


    //这一段很固定，还要记得在主循环加上循环条件和getendparam
   ros::param::set("/find_apriltag_node/start",false);
   ros::param::set("/find_apriltag_node/end",false);
   ros::Rate rate(10);
   bool start_param=false;
   bool end_param=false;
   ros::param::get("/find_apriltag_node/start",start_param);
   ros::param::get("/find_apriltag_node/end",end_param);
    while(!start_param && ros::ok())
    {
        ros::param::get("/find_apriltag_node/start",start_param);
        std::cout<<"waiting to start..."<<std::endl;
        rate.sleep();
    }
    ROS_INFO("start  sentting apriltag pixel!");


    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }


    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

    Mat frame, gray;
    while (!end_param && ros::ok()) 
    {
        ros::param::get("/find_apriltag_node/end",end_param);
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        if(zarray_size(detections)==0)
        {
            center_pixel.theta=0;
        }
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            // line(frame, Point(det->p[0][0], det->p[0][1]),
            //          Point(det->p[1][0], det->p[1][1]),
            //          Scalar(0, 0xff, 0), 2);
            // line(frame, Point(det->p[0][0], det->p[0][1]),
            //          Point(det->p[3][0], det->p[3][1]),
            //          Scalar(0, 0, 0xff), 2);
            // line(frame, Point(det->p[1][0], det->p[1][1]),
            //          Point(det->p[2][0], det->p[2][1]),
            //          Scalar(0xff, 0, 0), 2);
            // line(frame, Point(det->p[2][0], det->p[2][1]),
            //          Point(det->p[3][0], det->p[3][1]),
            //          Scalar(0xff, 0, 0), 2);
            int center_x=((det->p[0][0])+(det->p[1][0])+(det->p[2][0])+(det->p[3][0]))/4;
            int center_y=((det->p[0][1])+(det->p[1][1])+(det->p[2][1])+(det->p[3][1]))/4;
            // circle(frame,Point(center_x,center_y),5,Scalar(0,0,255),3,8);
            center_pixel.x=center_x;
            center_pixel.y=center_y;
            center_pixel.theta=1;
            std::cout<<"[find]"<<"("<<center_x<<","<<center_y<<")"<<std::endl;
            // stringstream ss;
            // ss << det->id;
            // String text = ss.str();
            // int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            // double fontscale = 1.0;
            // int baseline;
            // Size textsize = getTextSize(text, fontface, fontscale, 2,
            //                                 &baseline);
            // putText(frame, text, Point(det->c[0]-textsize.width/2,
            //                            det->c[1]+textsize.height/2),
            //         fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

            // First create an apriltag_detection_info_t struct using your known parameters.

            // apriltag_detection_info_t info;

            // float tagsize=0.1;
            // int fx=500;
            // int fy=500;
            // int cx=320;
            // int cy=240;
            // info.det = det;
            // info.tagsize = tagsize;
            // info.fx = fx;
            // info.fy = fy;
            // info.cx = cx;
            // info.cy = cy;

            // // Then call estimate_tag_pose.
            // apriltag_pose_t pose;
            // double err = estimate_tag_pose(&info, &pose);
            // // Do something with pose.
            // std::cout<<(pose.t)->data[0]<<","<<(pose.t)->data[1]<<","<<(pose.t)->data[2]<<std::endl;
        }
        apriltag_pixel_pub.publish(center_pixel);

        apriltag_detections_destroy(detections);
        //circle(frame,Point(WINDOW_W/2,WINDOW_H/2),5,Scalar(255,0,255),3,8);
        //imshow("Tag Detections", frame);
        //printf("1");
        //if (waitKey(30) >= 0)
        //    printf("2");
        //    break;
        waitKey(1);
    }

    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(famname, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }


    getopt_destroy(getopt);

    return 0;
}
