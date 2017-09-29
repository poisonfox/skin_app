#ifndef skin_H
#define skin_H

#define _USE_MATH_DEFINES

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "std_msgs/String.h"

using namespace cv;
using namespace std;

class skin {

    private:
        // ROS specific variables
        ros::NodeHandle 				nh;
        ros::Subscriber     			camerainfo_sub;
        image_transport::Subscriber     image_sub;
        image_transport::ImageTransport it;
        image_transport::Publisher      image_pub;
        image_transport::Publisher      image_pub_2;         

        // openCV variables
        cv::Mat 	origFrame;

    public:
        skin();
        virtual ~skin() { };

        //Functions
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        Mat setRoi(Mat run);
        Mat gaussian_blur_difference(cv::Mat run);
        Mat flood(Mat run);
        Mat deleteWhite(Mat run);
        void thinningIteration(Mat& img, int iter);
        void thinning(const Mat& src, Mat& dst);
        
        //Mat
        Mat gauss;                   // gaussianBlur
        Mat grayFrame;
        Mat image_bw;
        Mat flooded;
        Mat clean;
        Mat out;
        Mat thin;

};

#endif
