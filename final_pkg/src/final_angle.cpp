/************************************************************************
 * @brief	RobotChallenge/Image Processing exercise 01
 *
 * @file 	exercise04.cpp
 * @author	Benjamin Munske (Benjamin.Munske@imes.uni-hannover.de)
 * @date	02. Nov. 2011
 *
 * Gottfried Wilhelm Leibniz Universität
 * Institut für Mechatronische Systeme
 * Appelstraße 11a
 * 30167 Hannover
 *
 ************************************************************************/

#include <final_pkg/final_angle.h>

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <limits>
#include <fstream>
#include <cmath>
#include <sstream>
#include "std_msgs/String.h"
#include "ros/ros.h"

#include <sensor_msgs/fill_image.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>


using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

/**
 * @brief class constructor
 */
final_angle::final_angle() : it(nh) {
    ros::NodeHandle n;
	// create image output windows
	namedWindow("Original", CV_WINDOW_AUTOSIZE);


	startWindowThread();

    image_sub 		= it.subscribe("/usb_cam/image_raw",   1, &final_angle::imageCallback, this);
    theta_publisher_ = n.advertise<std_msgs::String>("theta",10);
}


/**
 * @brief This function is called, when a new image is received
 */
void final_angle::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;

	try {
        Mat grayFrame;
		Mat gray3CH;
        Mat dst, src;
        //char* window_name = "Original";

        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        this->origFrame = cv_ptr->image;

		/************************************************************************************
		(A) Aufg:	Zunächst ist die bereits bekannte Konvertierung des Eingangsbildes
					"this->origFrame" in ein Grauwertbild "grayFrame" durchzuführen.
					Anschließend ist erneut eine Gauß-Filterung mi der Filermatrixgröße 5 x 5
					durch zuführen.
					
					Die in diesem Beispiel verwendete Videosequenz zeigt Bälle, die mittels
					einer Hough-Transformaion detektiert werden sollen, siehe hierzu [1]. Lesen
					Sie sich vor der Implementierung das Beispiel [1] druch.
					
					Wandeln Sie anschließend das Grauwertbild in ein drei-Kanaliges BGR-Bild
					um [2], damit innerhalb der bereits implementierten for-Schleife, siehe
					unten, farbige Kreise in die Bildmarix eingezeichnet werden können. Als
					Bildmatrix für das drei-Kanalige Bild soll die Variable "gray3CH" verwendet
					werden.
					
					Bei der Ausführung des Programms werden Sie feststellen, dass die Ball-
					erkennung nicht robust ist. Überlegen Sie sich, wie sie mit bereits
					bekannten Mitteln die Robustheit steigern können. Diskutieren Sie dieses
					Themen im Hinblick auf den ChallengeDay 2 innerhalb Ihres Teams. 
					
        Hinweise:	[1] http://opencv.willowgarage.com/documentation/cpp/imgproc_feature_detection.html?highlight=houghlines#HoughLines
					[2] http://opencv.willowgarage.com/documentation/cpp/core_operations_on_arrays.html#cv-merge
		**************************************************************************************/
		/// ---->
			// convert to gray scale image
            cvtColor(this->origFrame, grayFrame, CV_BGR2GRAY);


			// apply a gaussian blur to the gray image
            GaussianBlur(grayFrame,grayFrame,cv::Size(5,5),1.5);

            // irgendwie versuchen zu filtern
     //       Mat element(6, 6, CV_8U,cv::Scalar(1));
     //       erode(grayFrame,grayFrame, element);
     //       dilate(grayFrame,grayFrame, element);

            // detect lines using hough
            Canny(grayFrame, grayFrame, 50, 200, 3);

            cv::Mat ROI = grayFrame(cv::Range(208,408),cv::Range(235,435));
            dst = Scalar::all(0);

            src.copyTo( dst, ROI);
            //imshow( window_name, dst );

            vector<Vec2f> lines;

            int cam_w, cam_h;

            //cam_w, cam_h = cv::get(grayFrame);


            HoughLines(ROI, lines, 1, CV_PI/180, 25, 0, 0 );

            for( size_t i = 0; i < lines.size(); i++ )
            {
               float rho = lines[i][0], theta = lines[i][1];
               Point pt1, pt2;
               double a = cos(theta), b = sin(theta);
               double x0 = a*rho, y0 = b*rho;
               pt1.x = cvRound(x0 + 1000*(-b));
               pt1.y = cvRound(y0 + 1000*(a));
               pt2.x = cvRound(x0 - 1000*(-b));
               pt2.y = cvRound(y0 - 1000*(a));

               std_msgs::String msg;
               std::stringstream ss;

               if(theta < 0.25*3.14){
                   ROS_INFO("%f",theta);
                   ss << theta;
                   msg.data = ss.str();
                   theta_publisher_.publish(msg);
               }
               else if(theta > 0.25*3.14 && theta < 0.75*3.14){
                   theta =  theta - 3.14*0.5;
                   ROS_INFO("%f",theta);
                   ss << theta;
                   msg.data = ss.str();
                   theta_publisher_.publish(msg);
               }
               else if(theta > 0.75*3.14 && theta < 1.25*3.14){
                   theta =  theta - 3.14;
                   ROS_INFO("%f",theta);
                   ss << theta;
                   msg.data = ss.str();
                   theta_publisher_.publish(msg);
               }
               line( gray3CH, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
             }
			// create a 3-channel image out of the 1-channel gray scale image
            Mat grayVec[3] = {ROI, ROI, ROI};
            merge(grayVec, 3, gray3CH);
        /// <---

        // draw the detected lines and cirle radii into the image
    /*    for( size_t i = 0; i < lines.size(); i++ ) {
            // draw lines
            Point center(cvRound(lines[i][0]), cvRound(lines[i][1]));
            int radius = cvRound(lines[i][2]);
            circle(gray3CH, center, radius, Scalar(0,255,0), 2, 8, 0 );

			// print radii
			stringstream ss;
			ss.str("");
			ss.precision(2);
			ss << "r = " << radius;
            putText(gray3CH, ss.str(), Point(center.x - 30, center.y + 3), FONT_HERSHEY_SIMPLEX, 0.50, Scalar(255, 255, 255), 1, CV_AA);
        }*/

		/************************************************************************************
		(B) Aufg:	Abschließend soll eine Bild-im-Bild Funktion erzeugt werden, bei der das
					Eingangsbild "this->origFrame" auf ein Viertel der ursprünglichen Auf-
					lösung [1] skaliert werden soll [2].
					
					Das skalierte Bild soll im oberen rechen Bereich des Bildes "gray3CH"
					platziert werden und das Bild "gray3CH" soll im Fenster "Original" ange-
					zeigt werden.
					
		Hinweise:	[1] http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html?highlight=size#Mat::size
					[2] http://opencv.willowgarage.com/documentation/cpp/imgproc_geometric_image_transformations.html
		**************************************************************************************/
		
		/// ---->
            // resize the original color image to 0.25 of it's original size
			Mat resizedImage;
			Size PIPsize = Size(this->origFrame.cols / 4, this->origFrame.rows / 4);
			resize(this->origFrame, resizedImage, PIPsize);

			// create a region of interest to which the resized color image is meant to be copied
            Mat PIP = gray3CH(Rect((gray3CH.cols - PIPsize.width - 10), 10, PIPsize.width, PIPsize.height));
    //        resizedImage.copyTo(PIP);

			// show the results
			imshow("Original", gray3CH);
        /// <----
    }
	catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
	}
}

/**
 * @brief main function
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "RobotChallenge_ImageProc01_Ex04");

    final_angle myImageClass;

	ros::spin();
}
