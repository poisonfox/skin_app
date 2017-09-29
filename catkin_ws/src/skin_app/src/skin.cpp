#include <skin_app/skin.h>

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

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

skin::skin() : it(nh) {
    
    ros::NodeHandle n;
    image_sub     = it.subscribe("/usb_cam/image_raw",   1, &skin::imageCallback, this);
    //image_sub     = it.subscribe("/pylon_camera_node/image_raw",   1, &skin::imageCallback, this);
    image_pub = it.advertise("/image_converter/output_video", 1);
    image_pub_2 = it.advertise("/image_converter/output_video", 1);
}

Mat skin::setRoi(Mat run){
    
    int x = 80;
    int y = 0;
    int w = 480;
    int h = 480;
    
    Rect roi = Rect(x, y, w, h);
    run = run(roi);

    return run;
}

Mat skin::gaussian_blur_difference(Mat run){

  Mat g_1, g_2;
  run.copyTo(g_1);
  run.copyTo(g_2);

  GaussianBlur(g_1, g_1, Size(3, 3), 1.5);
  GaussianBlur(g_2, g_2, Size(51, 51), 1.5);

  gauss = (2 * g_1 - g_2);

  return gauss;
}

Mat skin::flood(Mat run){
  
  Mat mask_flood, image_flooded;
  run.copyTo(mask_flood);
  run.copyTo(image_flooded);

  Scalar new_Val = 255;       // New value of the repainted domain pixels.
  Rect* rect = 0;           // Optional output parameter set by the function to the minimum bounding rectangle of the repainted domain.
  Scalar loDiff = 1;          // Maximal lower brightness/color difference between the currently observed pixel and one of its neighbors belonging to the component, or a seed pixel being added to the component.
  Scalar upDiff = 1;  

  // Searches trough every pixel in every column
  for (int i = 0; i < mask_flood.cols; i++) {

    // If pixel is black, 
    if (mask_flood.at<char>(0, i) == 0) {
      floodFill(mask_flood, Point(i, 0), new_Val, rect, loDiff, upDiff);
    }
    if (mask_flood.at<char>(mask_flood.rows - 1, i) == 0) {
      floodFill(mask_flood, Point(i, mask_flood.rows - 1), new_Val, rect, loDiff, upDiff);
    }
  }
  for (int i = 0; i < mask_flood.rows; i++) {
    if (mask_flood.at<char>(i, 0) == 0) {
      floodFill(mask_flood, Point(0, i), new_Val, rect, loDiff, upDiff);
    }
    if (mask_flood.at<char>(i, mask_flood.cols - 1) == 0) {
      floodFill(mask_flood, Point(mask_flood.cols - 1, i), new_Val, rect, loDiff, upDiff);
    }
  }

  // Compare mask_flood with original, fill up holes in input image
  for (int row = 0; row < mask_flood.rows; ++row) {
    for (int col = 0; col < mask_flood.cols; ++col) {
      if (mask_flood.at<char>(row, col) == 0) {
        image_flooded.at<char>(row, col) = 255;
      }
    }
  }

  return image_flooded;

}

Mat skin::deleteWhite(Mat run){

  for (int i = 1; i < run.rows - 1; ++i) {
    for (int j = 1; j < run.cols - 1; ++j){

      if ((run.at<uchar>(i, j) == 255) &&
        (run.at<uchar>(i - 1, j + 0) == 0) &&      // Top 
        (run.at<uchar>(i + 0, j - 1) == 0) &&      // Left
        (run.at<uchar>(i + 0, j + 1) == 0) &&      // Right
        (run.at<uchar>(i + 1, j + 0) == 0)){      // Bottom
        run.at<uchar>(i, j) = 0;
      }

    }
  }

  for (int i = 0; i < run.rows - 5; ++i) {
    for (int j = 0; j < run.cols - 5; ++j){
      if ((run.at<uchar>(i + 0, j + 0) == 0) &&
        (run.at<uchar>(i + 0, j + 1) == 0) &&
        (run.at<uchar>(i + 0, j + 2) == 0) &&
        (run.at<uchar>(i + 0, j + 3) == 0) &&
        (run.at<uchar>(i + 1, j + 0) == 0) &&
        (run.at<uchar>(i + 1, j + 3) == 0) &&
        (run.at<uchar>(i + 2, j + 0) == 0) &&
        (run.at<uchar>(i + 2, j + 3) == 0) &&
        (run.at<uchar>(i + 3, j + 0) == 0) &&
        (run.at<uchar>(i + 3, j + 1) == 0) &&
        (run.at<uchar>(i + 3, j + 2) == 0) &&
        (run.at<uchar>(i + 3, j + 3) == 0) &&
        ((run.at<uchar>(i + 1, j + 1) == 0) ||
        (run.at<uchar>(i + 1, j + 2) == 0) ||
        (run.at<uchar>(i + 2, j + 1) == 0) ||
        (run.at<uchar>(i + 2, j + 2) == 0))){
        run.at<uchar>(i + 1, j + 1) = 0;
        run.at<uchar>(i + 1, j + 2) = 0;
        run.at<uchar>(i + 2, j + 1) = 0;
        run.at<uchar>(i + 2, j + 2) = 0;

      }
    }
  }

      for (int i = 0; i < run.rows - 5; ++i) {
        for (int j = 0; j < run.cols - 5; ++j){
          if ((run.at<uchar>(i + 0, j + 0) == 0) &&
            (run.at<uchar>(i + 0, j + 1) == 0) &&
            (run.at<uchar>(i + 0, j + 2) == 0) &&
            (run.at<uchar>(i + 0, j + 3) == 0) &&
            (run.at<uchar>(i + 0, j + 4) == 0) &&
            (run.at<uchar>(i + 1, j + 0) == 0) &&
            (run.at<uchar>(i + 1, j + 4) == 0) &&
            (run.at<uchar>(i + 2, j + 0) == 0) &&
            (run.at<uchar>(i + 2, j + 4) == 0) &&
            (run.at<uchar>(i + 3, j + 0) == 0) &&
            (run.at<uchar>(i + 3, j + 4) == 0) &&
            (run.at<uchar>(i + 4, j + 0) == 0) &&
            (run.at<uchar>(i + 4, j + 1) == 0) &&
            (run.at<uchar>(i + 4, j + 2) == 0) &&
            (run.at<uchar>(i + 4, j + 3) == 0) &&
            (run.at<uchar>(i + 4, j + 4) == 0) &&
            ((run.at<uchar>(i + 1, j + 1) == 0) ||
            (run.at<uchar>(i + 1, j + 2) == 0) ||
            (run.at<uchar>(i + 1, j + 3) == 0) ||
            (run.at<uchar>(i + 2, j + 1) == 0) ||
            (run.at<uchar>(i + 2, j + 2) == 0) ||
            (run.at<uchar>(i + 1, j + 3) == 0) ||
            (run.at<uchar>(i + 3, j + 1) == 0) ||
            (run.at<uchar>(i + 3, j + 2) == 0) ||
            (run.at<uchar>(i + 3, j + 3) == 0))){
            run.at<uchar>(i + 1, j + 1) = 0;
            run.at<uchar>(i + 1, j + 2) = 0;
            run.at<uchar>(i + 1, j + 3) = 0;
            run.at<uchar>(i + 2, j + 1) = 0;
            run.at<uchar>(i + 2, j + 2) = 0;
            run.at<uchar>(i + 2, j + 3) = 0;
            run.at<uchar>(i + 3, j + 1) = 0;
            run.at<uchar>(i + 3, j + 2) = 0;
            run.at<uchar>(i + 3, j + 3) = 0;

          }
        }
      }
  return run;
}

void skin::thinningIteration(Mat& img, int iter){

  CV_Assert(img.channels() == 1);
  CV_Assert(img.depth() != sizeof(uchar));
  CV_Assert(img.rows > 3 && img.cols > 3);

  Mat marker = Mat::zeros(img.size(), CV_8UC1);

  int nRows = img.rows;
  int nCols = img.cols;

  if (img.isContinuous()) {
    nCols *= nRows;
    nRows = 1;
  }

  int x, y;
  uchar *pAbove;
  uchar *pCurr;
  uchar *pBelow;
  uchar *nw, *no, *ne;    // north (pAbove)
  uchar *we, *me, *ea;
  uchar *sw, *so, *se;    // south (pBelow)

  uchar *pDst;

  // initialize row pointers
  pAbove = NULL;
  pCurr = img.ptr<uchar>(0);
  pBelow = img.ptr<uchar>(1);

  for (y = 1; y < img.rows - 1; ++y) {
    // shift the rows up by one
    pAbove = pCurr;
    pCurr = pBelow;
    pBelow = img.ptr<uchar>(y + 1);

    pDst = marker.ptr<uchar>(y);

    // initialize col pointers
    no = &(pAbove[0]);
    ne = &(pAbove[1]);
    me = &(pCurr[0]);
    ea = &(pCurr[1]);
    so = &(pBelow[0]);
    se = &(pBelow[1]);

    for (x = 1; x < img.cols - 1; ++x) {
      // shift col pointers left by one (scan left to right)
      nw = no;
      no = ne;
      ne = &(pAbove[x + 1]);
      we = me;
      me = ea;
      ea = &(pCurr[x + 1]);
      sw = so;
      so = se;
      se = &(pBelow[x + 1]);

      int A = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) +
        (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) +
        (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
        (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
      int B = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
      int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
      int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

      if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
        pDst[x] = 1;
    }
  }

  img &= ~marker;
}

void skin::thinning(const Mat& src, Mat& dst){

  dst = src.clone();
  dst /= 255;         // convert to binary image

  Mat prev = Mat::zeros(dst.size(), CV_8UC1);
  Mat diff;

  do {
    thinningIteration(dst, 0);
    thinningIteration(dst, 1);
    cv::absdiff(dst, prev, diff);
    dst.copyTo(prev);
  } while (countNonZero(diff) > 0);

  dst *= 255;
}

void skin::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  
  //ROS_INFO("Received image with size: %i x %i", msg->width, msg->height);
  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImage out_msg;
  cv_bridge::CvImage out_msg_2;

  try {
    //ROS_INFO("try");

    //ROS Image message --> cv::Mat via cv_bridge
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8); this->origFrame = cv_ptr->image;

    //Set Region of Interetest to 480 x 480 px
    this->origFrame = setRoi(this->origFrame);

    //Noise removal, build difference of gaussian filtered images
    gauss = gaussian_blur_difference(this->origFrame);

    //Convert to grayscale image
    cvtColor(gauss, grayFrame, CV_BGR2GRAY);

    //Adaptive Threshold
    adaptiveThreshold(grayFrame, image_bw, 255, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, -1);

    // Morph
    dilate(image_bw,image_bw, getStructuringElement(MORPH_RECT, Size(2, 2)));
    erode(image_bw, image_bw, getStructuringElement(MORPH_RECT, Size(2, 2)));

    erode(image_bw, image_bw, getStructuringElement(MORPH_RECT, Size(2, 2)));
    dilate(image_bw, image_bw, getStructuringElement(MORPH_RECT, Size(2, 2)));

    //
    flooded = flood(image_bw);

    //
    clean = deleteWhite(flooded);

    //Skeletonizing  
    thinning((255-clean), thin);
    Mat inv = (255 - thin);

    // Output
    out_msg.image = inv; 
    out_msg_2.image = image_bw;
    cv::imshow("Skeleton", inv);
    cv::imshow("Binary", image_bw);
    cv::waitKey(3);
    image_pub.publish(out_msg.toImageMsg());
    image_pub_2.publish(out_msg_2.toImageMsg());

    }

  catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "skin_feature");

    skin myImageClass;

  ros::spin();
}
