/**
*
* featureDetect: This file reads the pr2 right and left wide stereo camera images and performs feature detection
* 
*
* @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2012.
* @contact isura.ranatunga@mavs.uta.edu
* @see http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
* @created 11/28/2012
* @modified 11/28/2012
*
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class FeatureDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber R_image_sub_;
  image_transport::Publisher R_image_pub_;
  image_transport::Subscriber L_image_sub_;
  image_transport::Publisher L_image_pub_; 

public:
  FeatureDetector()
    : it_(nh_)
  {
    R_image_pub_ = it_.advertise("R_out", 1);//need to change this topic name
    R_image_sub_ = it_.subscribe("/wide_stereo/right/image_color", 1, &FeatureDetector::R_imageCb, this);
    L_image_pub_ = it_.advertise("L_out", 1);//need to change this topic name
    L_image_sub_ = it_.subscribe("/wide_stereo/left/image_color", 1, &FeatureDetector::L_imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~FeatureDetector()
  {
    cv::destroyWindow(WINDOW);
  }

  void R_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat R_img;//standard OpenCV image format

    //SIFT
    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      R_img = cv_ptr->image;
      detector.detect(R_img, keypoints);//detect SIFT features
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Add results to image and save.
    cv::Mat output;
    cv::drawKeypoints(R_img, keypoints, output);
    cv_ptr->image = output;

    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    //cv::imshow(WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    R_image_pub_.publish(cv_ptr->toImageMsg());
  }

void L_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat L_img;//standard OpenCV image format

    //SIFT
    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      L_img = cv_ptr->image;
      detector.detect(L_img, keypoints);//detect SIFT features
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Add results to image and save.
    cv::Mat output;
    cv::drawKeypoints(L_img, keypoints, output);
    cv_ptr->image = output;

    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    //cv::imshow(WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    L_image_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_detector");
  FeatureDetector detect;
  ros::spin();
  return 0;
}
