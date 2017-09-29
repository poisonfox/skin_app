#ifndef FINAL_ANGLE_H
#define FINAl_ANGLE_H

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "std_msgs/String.h"



/**
 * @class final_angle
 */
class final_angle {
    private:
        // ROS specific variables
        ros::NodeHandle 				nh;
        ros::Subscriber     			camerainfo_sub;
        ros::Publisher                  theta_publisher_;
        image_transport::Subscriber     image_sub;
        image_transport::ImageTransport it;

        // openCV variables
        cv::Mat 	origFrame;		///< original frame

    public:
        final_angle();
        virtual ~final_angle() { };

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif
