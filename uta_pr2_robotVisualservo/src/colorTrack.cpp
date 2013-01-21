/**
*
* colorTrack: This file reads the drc robot right and left wide stereo camera images and tracks red, green. blue and yellow blobs.
* This node outputs a message containing the pixel locations of the four blobs, this is not very robust and assumes that the input is four blobs that are red, green, blue and yellow.
* 
* @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2012.
* @contact isura.ranatunga@mavs.uta.edu
* @see http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
* @created 12/03/2012
* @modified 12/04/2012
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
#include <stdio.h>
#include <iostream>
#include <fstream>

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

  //Create file pointer
  ofstream datacollectionFile;
  cv::Vec3f V_prev;
  typedef cv::Vec<float, 8> Vec8f;
  Vec8f p_prev;

  float Z;


public:
  FeatureDetector()
    : it_(nh_)
  {
    cmd_vel = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 10);
    imgFeature = nh_.advertise<uta_pr2_robotVisualservo::imgFeature>("right/imageFeature", 10);
    R_image_pub_ = it_.advertise("R_out", 1);//need to change this topic name
    R_image_sub_ = it_.subscribe("/wide_stereo/right/image_color", 1, &FeatureDetector::R_imageCb, this);
    //R_imageinfo_sub_ = it_.subscribe("/wide_stereo/right/image_color", 1, &FeatureDetector::R_imageinfoCallback, this);
    L_image_pub_ = it_.advertise("L_out", 1);//need to change this topic name
    L_image_sub_ = it_.subscribe("/wide_stereo/left/image_color", 1, &FeatureDetector::L_imageCb, this);
    //L_imageinfo_sub_ = it_.subscribe("/wide_stereo/left/image_color", 1, &FeatureDetector::L_RimageinfoCallback, this);
    head_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("head_traj_controller/command", 20);

    cv::namedWindow(WINDOW);

    req_tilt = 0.0; req_tilt_vel = 0.0; max_tilt = 1.4; min_tilt = -0.4; tilt_step = 0.015;

    //Data collection
    datacollectionFile.open ("IBVS_lam_-#_Z_#.txt");

    //initial V
    FeatureDetector::V_prev[0] = 0;
    FeatureDetector::V_prev[1] = 0;
    FeatureDetector::V_prev[2] = 0;

    Z = 5.0;

  }

  ~FeatureDetector()
  {
    cv::destroyWindow(WINDOW);
    datacollectionFile.close();
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

    /*/// Draw contours
    cv::Mat drawing = cv::Mat::zeros( redImg.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
       {
         cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
         cv::circle( drawing, mcR[i], 4, color, -1, 8, 0 );
       }*/

    /// Find contours for GREEN
    cv::findContours( greenImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Get the moments
    //cv::vector<cv::Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
       { mu[i] = cv::moments( contours[i], false ); }

    ///  Get the mass centers:
    cv::vector<cv::Point2f> mcG( contours.size() );
    //If there are no contours the center of mass is Zero
    //if( contours.size() == 0 ){cv::vector<cv::Point2f> mcG( 0 );mcG[0].x=0;mcG[0].y=0;}
    for( int i = 0; i < contours.size(); i++ )
       { mcG[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

    /// Find contours for BLUE
    cv::findContours( blueImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Get the moments
    //cv::vector<cv::Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
       { mu[i] = cv::moments( contours[i], false ); }

    ///  Get the mass centers:
    cv::vector<cv::Point2f> mcB( contours.size() );
    //If there are no contours the center of mass is Zero
    //if( contours.size() == 0 ){cv::vector<cv::Point2f> mcB( 0 );mcB[0].x=0;mcB[0].y=0;}
    for( int i = 0; i < contours.size(); i++ )
       { mcB[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }


    /// Find contours for YELLOW
    cv::findContours( yellowImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Get the moments
    //cv::vector<cv::Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
       { mu[i] = cv::moments( contours[i], false ); }

    ///  Get the mass centers:
    cv::vector<cv::Point2f> mcY( contours.size() );
    //If there are no contours the center of mass is Zero
    //if( contours.size() == 0 ){cv::vector<cv::Point2f> mcY( 0 );mcY[0].x=0;mcY[0].y=0;}
    for( int i = 0; i < contours.size(); i++ )
       { mcY[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }


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


    /////////////////////////////////////////////////////////////////////////
    // Calculate the Visual Servoing - Will be moved to new node in future //
    /////////////////////////////////////////////////////////////////////////    
    
    // Proportional gain
    float lambda = -0.5;

    //K: [320.0, 0.0, 320.5, 0.0, 320.0, 240.5, 0.0, 0.0, 1.0]
    float fKu = 320.0; float fKv = 320.5; float u0 = 320.0; float v0 = 240.5;
    float Z = FeatureDetector::Z;
    Z = 2;

    // Desired positions of 4 points
    // RGBY
    // Old: 265.000000 208.000000, 329.308167 207.735855, 256.815491 256.089294, 331.157959 254.868118
    // 273.681488 189.617279, 337.738983 190.915100, 264.134369 237.941864, 338.809509 238.875000
    typedef cv::Vec<float, 8> Vec8f;
    //there are 4 J matrices one for each point
    typedef cv::Matx<float, 8, 3> Matx83f;
    typedef cv::Matx<float, 3, 8> Matx38f;
    typedef cv::Matx<float, 8, 2> Matx82f;
    typedef cv::Matx<float, 8, 1> Matx81f;
    Vec8f p_des, p_curr, p_err, p_dot;
    p_des[0]=273.681488;p_des[1]=189.617279;p_des[2]=337.738983;p_des[3]=190.915100;p_des[4]=264.134369;p_des[5]=237.941864;p_des[6]=338.809509;p_des[7]=238.875000;
    p_curr[0]=mcR[0].x;p_curr[1]=mcR[0].y;p_curr[2]=mcG[0].x;p_curr[3]=mcG[0].y;p_curr[4]=mcB[0].x;p_curr[5]=mcB[0].y;p_curr[6]=mcY[0].x;p_curr[7]=mcY[0].y;

    //p_prev[0]=FeatureDetector::p_prev[0];p_prev[1]=FeatureDetector::p_prev[1];p_prev[2]=FeatureDetector::p_prev[2];p_prev[3]=FeatureDetector::p_prev[3];
    //p_prev[4]=FeatureDetector::p_prev[4];p_prev[5]=FeatureDetector::p_prev[5];p_prev[6]=FeatureDetector::p_prev[6];p_prev[7]=FeatureDetector::p_prev[7];

    p_err = p_des-p_curr;
    p_dot = p_curr-FeatureDetector::p_prev;

    FeatureDetector::p_prev[0]=p_curr[0];FeatureDetector::p_prev[1]=p_curr[1];FeatureDetector::p_prev[2]=p_curr[2];FeatureDetector::p_prev[3]=p_curr[3];
    FeatureDetector::p_prev[4]=p_curr[4];FeatureDetector::p_prev[5]=p_curr[5];FeatureDetector::p_prev[6]=p_curr[6];FeatureDetector::p_prev[7]=p_curr[7];

    //ROS_INFO("Contours: %d | ERR: %f %f, %f %f, %f %f, %f %f", contours.size(), p_err[0],p_err[1],p_err[2],p_err[3],p_err[4],p_err[5],p_err[6],p_err[7]);
    
    //Saving errors to file
    FeatureDetector::datacollectionFile << p_err[0]<<", "<<p_err[1]<<", "<<p_err[2]<<", "<<p_err[3]<<", "<<p_err[4]<<", "<<p_err[5]<<", "<<p_err[6]<<", "<<p_err[7]<<"\n";

    //Draw P_desired on image
    cv::circle( cv_ptr->image, cv::Point(p_des[0],p_des[1]), 4, CV_RGB(255,0,0),3);
    cv::circle( cv_ptr->image, cv::Point(p_des[2],p_des[3]), 4, CV_RGB(0,255,0),3);
    cv::circle( cv_ptr->image, cv::Point(p_des[4],p_des[5]), 4, CV_RGB(0,0,255),3);
    cv::circle( cv_ptr->image, cv::Point(p_des[6],p_des[7]), 4, CV_RGB(255,255,0),3);

    /*
    // Creating Ja matrix only Vx, Vy, Wx and Wz are used
    Matx84f J_p; Matx48f J_p_INV;
    //Red
    J_p(0,0)= -fKu/Z; J_p(0,1)= 0;      J_p(0,2)= (mcR[0].x-u0)*(mcR[0].y-v0)/fKu;         J_p(0,3)= (mcR[0].y-v0); 
    J_p(1,0)= 0;      J_p(1,1)= -fKv/Z; J_p(1,2)= fKv+((mcR[0].y-v0)*(mcR[0].y-v0))/fKv;   J_p(1,3)= -(mcR[0].x-u0); 
    //Green
    J_p(2,0)= -fKu/Z; J_p(2,1)= 0;      J_p(2,2)= (mcG[0].x-u0)*(mcG[0].y-v0)/fKu;         J_p(2,3)= (mcG[0].y-v0); 
    J_p(3,0)= 0;      J_p(3,1)= -fKv/Z; J_p(3,2)= fKv+((mcG[0].y-v0)*(mcG[0].y-v0))/fKv;   J_p(3,3)= -(mcG[0].x-u0); 
    //Blue
    J_p(4,0)= -fKu/Z; J_p(4,1)= 0;      J_p(4,2)= (mcB[0].x-u0)*(mcB[0].y-v0)/fKu;         J_p(4,3)= (mcB[0].y-v0); 
    J_p(5,0)= 0;      J_p(5,1)= -fKv/Z; J_p(5,2)= fKv+((mcB[0].y-v0)*(mcB[0].y-v0))/fKv;   J_p(5,3)= -(mcB[0].x-u0); 
    //Yellow
    J_p(6,0)= -fKu/Z; J_p(6,1)= 0;      J_p(6,2)= (mcY[0].x-u0)*(mcY[0].y-v0)/fKu;         J_p(6,3)= (mcY[0].y-v0);
    J_p(7,0)= 0;      J_p(7,1)= -fKv/Z; J_p(7,2)= fKv+((mcY[0].y-v0)*(mcY[0].y-v0))/fKv;   J_p(7,3)= -(mcY[0].x-u0);
    */
    /*
    // Creating Ja matrix only Vx, Vy, Wy and Wz are used
    Matx84f J_p; Matx48f J_p_INV;
    //Red
    J_p(0,0)= -fKu/Z; J_p(0,1)= 0;      J_p(0,2)= -(fKu+(mcR[0].x-u0)*(mcR[0].x-u0)/fKu);  J_p(0,3)= (mcR[0].y-v0); 
    J_p(1,0)= 0;      J_p(1,1)= -fKv/Z; J_p(1,2)= -((mcR[0].x-u0)*(mcR[0].y-v0))/fKv;      J_p(1,3)= -(mcR[0].x-u0); 
    //Green
    J_p(2,0)= -fKu/Z; J_p(2,1)= 0;      J_p(2,2)= -(fKu+(mcG[0].x-u0)*(mcG[0].x-u0)/fKu);  J_p(2,3)= (mcG[0].y-v0); 
    J_p(3,0)= 0;      J_p(3,1)= -fKv/Z; J_p(3,2)= -((mcG[0].x-u0)*(mcG[0].y-v0))/fKv;      J_p(3,3)= -(mcG[0].x-u0); 
    //Blue
    J_p(4,0)= -fKu/Z; J_p(4,1)= 0;      J_p(4,2)= -(fKu+(mcB[0].x-u0)*(mcB[0].x-u0)/fKu);  J_p(4,3)= (mcB[0].y-v0); 
    J_p(5,0)= 0;      J_p(5,1)= -fKv/Z; J_p(5,2)= -((mcB[0].x-u0)*(mcB[0].y-v0))/fKv;      J_p(5,3)= -(mcB[0].x-u0); 
    //Yellow
    J_p(6,0)= -fKu/Z; J_p(6,1)= 0;      J_p(6,2)= -(fKu+(mcY[0].x-u0)*(mcY[0].x-u0)/fKu);  J_p(6,3)= (mcY[0].y-v0);
    J_p(7,0)= 0;      J_p(7,1)= -fKv/Z; J_p(7,2)= -((mcY[0].x-u0)*(mcY[0].y-v0))/fKv;      J_p(7,3)= -(mcY[0].x-u0);
    */
    
    // Creating Ja matrix only Vx, Vy and Wz are used
    Matx83f J_p; Matx38f J_p_INV;
    //Red
    J_p(0,0)= -fKu/Z; J_p(0,1)= 0;      J_p(0,2)= (mcR[0].y-v0); 
    J_p(1,0)= 0;      J_p(1,1)= -fKv/Z; J_p(1,2)= -(mcR[0].x-u0); 
    //Green
    J_p(2,0)= -fKu/Z; J_p(2,1)= 0;      J_p(2,2)= (mcG[0].y-v0); 
    J_p(3,0)= 0;      J_p(3,1)= -fKv/Z; J_p(3,2)= -(mcG[0].x-u0); 
    //Blue
    J_p(4,0)= -fKu/Z; J_p(4,1)= 0;      J_p(4,2)= (mcB[0].y-v0); 
    J_p(5,0)= 0;      J_p(5,1)= -fKv/Z; J_p(5,2)= -(mcB[0].x-u0); 
    //Yellow
    J_p(6,0)= -fKu/Z; J_p(6,1)= 0;      J_p(6,2)= (mcY[0].y-v0);
    J_p(7,0)= 0;      J_p(7,1)= -fKv/Z; J_p(7,2)= -(mcY[0].x-u0);
    
    cv::invert(J_p, J_p_INV, cv::DECOMP_SVD);
    //cv::Vec3f V = FeatureDetector::V_prev;
    cv::Vec3f V = lambda*J_p_INV*p_err;

    // Estimate Z
    cv::Vec2f V_v;
    V_v[0] = V[1];
    V_v[1] = V[0];

    // Creating J_t and J_w matrices only Vx, Vy and Wz are used
    Matx82f J_t; Matx81f J_w;
    //Red
    J_t(0,0)= -fKu;   J_t(0,1)= 0;
    J_t(1,0)= 0;      J_t(1,1)= -fKv;
    //Green
    J_t(2,0)= -fKu;   J_t(2,1)= 0;
    J_t(3,0)= 0;      J_t(3,1)= -fKv;
    //Blue
    J_t(4,0)= -fKu;   J_t(4,1)= 0;
    J_t(5,0)= 0;      J_t(5,1)= -fKv;
    //Yellow
    J_t(6,0)= -fKu;   J_t(6,1)= 0;
    J_t(7,0)= 0;      J_t(7,1)= -fKv;

    J_w(0,2)= (mcR[0].y-v0);
    J_w(1,2)= -(mcR[0].x-u0);
    //Green
    J_w(2,2)= (mcG[0].y-v0);
    J_w(3,2)= -(mcG[0].x-u0);
    //Blue
    J_w(4,2)= (mcB[0].y-v0);
    J_w(5,2)= -(mcB[0].x-u0);
    //Yellow
    J_w(6,2)= (mcY[0].y-v0);
    J_w(7,2)= -(mcY[0].x-u0);
    
    cv::Mat A(J_t*V_v);
    cv::Mat b(p_dot-J_w*V[2]);

    //Lease squares
    cv::Mat Zinv = (A.t()*A).inv()*(A.t()*b);
    ROS_INFO("Zinv: %f", FeatureDetector::Z);
    FeatureDetector::Z = 1/Zinv.at<float>(0,0);

    // create twist msg
    geometry_msgs::Twist twist;
    // linear
    twist.linear.x = V[1];//Important x and y switched
    twist.linear.y = V[0];
    twist.linear.z = 0;

    // angular
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = V[2];

    // send twist
    cmd_vel.publish(twist);

        /*
        //PR2 Head Control
        double dt = 1.0/double(PUBLISH_FREQ);
        double horizon = 3.0 * dt;

        req_tilt_vel = V[2];

        trajectory_msgs::JointTrajectory traj;
        traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
        traj.joint_names.push_back("head_pan_joint");
        traj.joint_names.push_back("head_tilt_joint");
        traj.points.resize(1);
        traj.points[0].positions.push_back(0);
        traj.points[0].velocities.push_back(0);
        traj.points[0].positions.push_back(req_tilt + req_tilt_vel * horizon);
        traj.points[0].velocities.push_back(req_tilt_vel);
        traj.points[0].time_from_start = ros::Duration(horizon);
        head_pub_.publish(traj);

        // Updates the current positions
        //req_pan += req_pan_vel * dt;
        //req_pan = max(min(req_pan, max_pan), -max_pan);
        req_tilt += req_tilt_vel * dt;
        req_tilt = max(min(req_tilt, max_tilt), min_tilt);
        */

    FeatureDetector::V_prev = V;

    cv::imshow(WINDOW, cv_ptr->image);
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
