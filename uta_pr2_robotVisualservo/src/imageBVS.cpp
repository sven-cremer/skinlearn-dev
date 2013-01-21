/**
*
* imageBVS: This node uses the features detected by either colorTrack or featureDetect and performs IBVS.
* This generates a Twist msg.
*
* @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2012.
* @contact isura.ranatunga@mavs.uta.edu
* @see http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
* @created 12/01/2012
* @modified 12/01/2012
*
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <uta_pr2_robotVisualservo/imgFeature.h>

class imageBVS
{
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel;


public:
  imageBVS()
  {
    cmd_vel = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 10);//cmd_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  }

  ~FeatureDetector()
  {

  }
  
  void featuresIBVS(const sensor_msgs::ImageConstPtr& msg)
  {

    /////////////////////////////////////////////////////////////////////////
    // Calculate the Visual Servoing - Will be moved to new node in future //
    /////////////////////////////////////////////////////////////////////////    
    
    // Proportional gain
    float lambda = -0.1;

    //K: [320.0, 0.0, 320.5, 0.0, 320.0, 240.5, 0.0, 0.0, 1.0]
    float fKu = 320.0; float fKv = 320.5; float u0 = 320.0; float v0 = 240.5;
    float Z = 2;

    // Desired positions of 4 points
    // RGBY
    // 265.000000 208.000000, 329.308167 207.735855, 256.815491 256.089294, 331.157959 254.868118
    typedef cv::Vec<float, 8> Vec8f;
    //there are 4 J matrices one for each point
    typedef cv::Matx<float, 8, 3> Matx84f;
    typedef cv::Matx<float, 3, 8> Matx48f;
    Vec8f p_des;Vec8f p_curr;Vec8f p_err;
    p_des[0]=265.000000;p_des[1]=208.000000;p_des[2]=329.308167;p_des[3]=207.735855;p_des[4]=256.815491;p_des[5]=256.089294;p_des[6]=331.157959;p_des[7]=254.868118;
    p_curr[0]=mcR[0].x;p_curr[1]=mcR[0].y;p_curr[2]=mcG[0].x;p_curr[3]=mcG[0].y;p_curr[4]=mcB[0].x;p_curr[5]=mcB[0].y;p_curr[6]=mcY[0].x;p_curr[7]=mcY[0].y;
    p_err = p_des-p_curr;
    ROS_INFO("Contours: %d | ERR: %f %f, %f %f, %f %f, %f %f", contours.size(), p_err[0],p_err[1],p_err[2],p_err[3],p_err[4],p_err[5],p_err[6],p_err[7]);
    
    //Draw P_desired on image
    cv::circle( cv_ptr->image, cv::Point(p_des[0],p_des[1]), 4, CV_RGB(255,0,0),3);
    cv::circle( cv_ptr->image, cv::Point(p_des[2],p_des[3]), 4, CV_RGB(0,255,0),3);
    cv::circle( cv_ptr->image, cv::Point(p_des[4],p_des[5]), 4, CV_RGB(0,0,255),3);
    cv::circle( cv_ptr->image, cv::Point(p_des[6],p_des[7]), 4, CV_RGB(255,255,0),3);

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
    Matx84f J_p; Matx48f J_p_INV;
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


    cv::Vec3f V;
    
    cv::invert(J_p, J_p_INV, cv::DECOMP_SVD);
    V = lambda*J_p_INV*p_err;

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
  ros::init(argc, argv, "feature_detector");
  FeatureDetector detect;
  ros::spin();
  return 0;
}

