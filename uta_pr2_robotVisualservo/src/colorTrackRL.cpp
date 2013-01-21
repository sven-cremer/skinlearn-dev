/**
*
* colorTrack: This file reads the pr2 right and left wide stereo camera images and tracks red, green. blue and yellow blobs.
* This node outputs a message containing the pixel locations of the four blobs, this is not very robust and assumes that the input is four blobs that are red, green, blue and yellow.
* 
* @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2012.
* @contact isura.ranatunga@mavs.uta.edu
* @see http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
* @created 12/03/2012
* @modified 12/03/2012
*
*/

#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <uta_pr2_robotVisualservo/imgFeature.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <math.h>

namespace enc = sensor_msgs::image_encodings;
using namespace std;

static const char WINDOW[] = "Image window";
const int PUBLISH_FREQ = 20;

class FeatureDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber R_image_sub_;
  //image_transport::Subscriber R_imageinfo_sub_;
  image_transport::Publisher R_image_pub_;
  image_transport::Subscriber L_image_sub_;
  //image_transport::Subscriber L_imageinfo_sub_;
  image_transport::Publisher L_image_pub_;
  ros::Publisher cmd_vel;
  ros::Publisher imgFeature;
  ros::Publisher head_pub_;

  double req_tilt,req_tilt_vel, max_tilt, min_tilt,tilt_step;


public:
  FeatureDetector()
    : it_(nh_)
  {
    cmd_vel = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 10);//cmd_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    imgFeature = nh_.advertise<uta_pr2_robotVisualservo::imgFeature>("right/imageFeature", 10);
    R_image_pub_ = it_.advertise("R_out", 1);//need to change this topic name
    R_image_sub_ = it_.subscribe("/gscam/image_raw", 1, &FeatureDetector::R_imageCb, this);
    //R_imageinfo_sub_ = it_.subscribe("/wide_stereo/right/image_color", 1, &FeatureDetector::R_imageinfoCallback, this);
    L_image_pub_ = it_.advertise("L_out", 1);//need to change this topic name
    L_image_sub_ = it_.subscribe("/wide_stereo/left/image_color", 1, &FeatureDetector::L_imageCb, this);
    //L_imageinfo_sub_ = it_.subscribe("/wide_stereo/left/image_color", 1, &FeatureDetector::L_RimageinfoCallback, this);
    head_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("head_traj_controller/command", 20);

    cv::namedWindow(WINDOW);

    req_tilt = 0.0; req_tilt_vel = 0.0; max_tilt = 1.4; min_tilt = -0.4; tilt_step = 0.015;
  }

  ~FeatureDetector()
  {
    cv::destroyWindow(WINDOW);
  }

  //void FeatureDetector::R_imageinfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info) { }
  //void FeatureDetector::L_imageinfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info) { }
  
  void R_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    //standard OpenCV image format
    cv::Mat R_img;
    cv::Mat redImg;
    cv::Mat greenImg;
    cv::Mat blueImg;
    cv::Mat yellowImg;
    cv::Mat rgbyImg;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Transform image to HSV
    cv::cvtColor(cv_ptr->image, R_img, CV_BGR2HSV);
    
    // Important to note: OpenCV stores the H component in HSV from 0 to 179
    cv::inRange(R_img,cv::Scalar(0,100,100),cv::Scalar(5,255,255),redImg);
    cv::inRange(R_img,cv::Scalar(55,100,100),cv::Scalar(65,255,255),greenImg);
    cv::inRange(R_img,cv::Scalar(100,100,100),cv::Scalar(120,255,255),blueImg);
    cv::inRange(R_img,cv::Scalar(20,100,100),cv::Scalar(30,255,255),yellowImg);

    /*//Erode image
    int erosion_elem = 0;
    int erosion_size = 1;
    int erosion_type;
    if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }
    cv::Mat element = cv::getStructuringElement( erosion_type,
                                     cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                     cv::Point( erosion_size, erosion_size ) );
    cv::erode( redImg, redImg, element );
    cv::erode( greenImg, greenImg, element );
    cv::erode( blueImg, blueImg, element );
    cv::erode( yellowImg, yellowImg, element );
    */

    //Median Filter
    cv::medianBlur(redImg, redImg, 7);
    cv::medianBlur(greenImg, greenImg, 7);
    cv::medianBlur(blueImg, blueImg, 7);
    cv::medianBlur(yellowImg, yellowImg, 7);

    rgbyImg =redImg+greenImg+blueImg+yellowImg;
    redImg = rgbyImg;
    cv::vector<cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    cv::RNG rng(12345);

    /// Find contours for RED
    cv::findContours( redImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Get the moments
    cv::vector<cv::Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
       { mu[i] = cv::moments( contours[i], false ); }

    ///  Get the mass centers:
    cv::vector<cv::Point2f> mcR( contours.size() );
    //If there are no contours the center of mass is Zero
    //if( contours.size() == 0 ){cv::vector<cv::Point2f> mcR( 0 );mcR[0].x=0;mcR[0].y=0;}
    for( int i = 0; i < contours.size(); i++ )
       { mcR[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

    /// Draw contours
    cv::Mat drawing = cv::Mat::zeros( redImg.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
       {
         cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
         cv::circle( drawing, mcR[i], 4, color, -1, 8, 0 );
       }




    //ROS_INFO("Contours: %d | POS: %f %f, %f %f, %f %f, %f %f", contours.size(), mcR[0].x, mcR[0].y, mcG[0].x, mcG[0].y, mcB[0].x, mcB[0].y, mcY[0].x, mcY[0].y);
    //cout << contours.size() << pt.x << ", " << pt.y << endl;    
    
    //cv_ptr->image = redImg;

    R_image_pub_.publish(cv_ptr->toImageMsg());

   /*
    //TODO Need to get this from image info
    //K: [320.0, 0.0, 320.5, 0.0, 320.0, 240.5, 0.0, 0.0, 1.0]
    float fKu = 320.0; float fKv = 320.5; float u0 = 320.0; float v0 = 240.5;
    float Z = 2;
    
    // create imgFeature msg
    uta_pr2_robotVisualservo::imgFeature imgF;
    imgF.points = {0};
    imgF.fKu = fKu;
    imgF.fKv = fKv;
    imgF.u0 = u0;
    imgF.v0 = v0;
    imgF.Z = Z;

    // send imgF
    imgFeature.publish(imgF);
*/

    cv::imshow(WINDOW, drawing);
    cv::waitKey(3);

  }

void L_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat L_img;//standard OpenCV image format

    //SIFT
    //cv::SiftFeatureDetector detector;
    //std::vector<cv::KeyPoint> keypoints;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      //L_img = cv_ptr->image;
      //detector.detect(L_img, keypoints);//detect SIFT features
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Add results to image and save.
    //cv::Mat output;
    //cv::drawKeypoints(L_img, keypoints, output);
    //cv_ptr->image = output;

    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    //cv::imshow(WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    L_image_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "colorTrack");
  FeatureDetector detect;
  ros::spin();
  return 0;
}
