#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;

#include <stdlib.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
using namespace std;

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "AprilTags/QRCode.h"
#include "std_msgs/String.h"
#define M100_CAMERA 1
#define VIDEO_STREAM 2
#define CURRENT_IMAGE_SOURCE VIDEO_STREAM
//#define CURRENT_IMAGE_SOURCE M100_CAMERA
#define VISABILITY false

/**global publisher*/
ros::Publisher vision_base_pub;
ros::Subscriber base_change_sub;
image_transport::Subscriber vision_image_sub;

QRCode qr_code;
std::stringstream ss;
cv::Mat g_image;
bool g_is_new_image= false;
bool g_is_base_running= true;
float g_height= 2.4;

void baseChangeCallback(const std_msgs::String::ConstPtr& msg);
void guidance_distance_callback(const sensor_msgs::LaserScan& g_oa);
/**global video capture and image*/
// cv::Mat g_pillar_image;
// cv::Mat g_line_image;
// cv::Mat g_base_image;
// int g_processed_time = 0;
/**callback of timer*/
// void cap_timer_callback( const ros::TimerEvent &evt );
// void pillar_timer_callback( const ros::TimerEvent &evt );
// void line_timer_callback( const ros::TimerEvent &evt );
// void base_timer_callback( const ros::TimerEvent &evt );

void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  g_image= cv_ptr->image;
  g_is_new_image= true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_challenge_qrcode_node");
  ros::NodeHandle node;

  vision_base_pub= node.advertise<std_msgs::String>("tpp/base", 1);

  qr_code.setVisability(false);
  qr_code.setup();

  image_transport::ImageTransport image_transport(node);
  vision_image_sub= image_transport.subscribe("m100/image", 1, imageCallBack);

  base_change_sub= node.subscribe("/tpp/base_change", 1, baseChangeCallback);
  ros::Subscriber guidance_distance_sub= node.subscribe(
      "/guidance/obstacle_distance", 1, guidance_distance_callback);
  /*main loop*/
  // cv::Mat m_copy = m_origin.clone();
  // cv::cvtColor(m_copy, m_copy, CV_BGR2HSV);
  // cv::imshow("copy", m_copy);
  // cv::waitKey(1);
  while(ros::ok())
  {
    /*read available image*/
    ros::spinOnce();

    if(g_image.empty())
      continue;

    if(!g_is_new_image)
      continue;

    if(g_is_base_running)
    {
      bool base_found;
      float base_direction_degree= 0;
      // ROS_INFO_STREAM("before");
      if(qr_code.getBasePosition(g_image, g_height))
      {
        ROS_INFO_STREAM("base position :" << qr_code.getBaseX() << " "
                                          << qr_code.getBaseY());
        base_found= true;
      }
      else
      {
        ROS_INFO_STREAM("can't find base");
        ROS_INFO_STREAM("base position :" << qr_code.getBaseX() << " "
                                          << qr_code.getBaseY());
        base_found= false;
      }
      // ROS_INFO_STREAM("after");
      qr_code.getBaseDirection(base_direction_degree);
      ROS_INFO_STREAM("base direction:" << base_direction_degree);

      ss.str("");
      std_msgs::String base_msg;
      ss << base_found << " " << qr_code.getBaseX() << " " << qr_code.getBaseY()
         << " " << base_direction_degree;
      base_msg.data= ss.str();
      vision_base_pub.publish(base_msg);

      g_is_new_image= false;
    }

    cv::waitKey(1);
  }

  return 1;
}

void baseChangeCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("receive base change info");
  if(msg->data == "pause")
    g_is_base_running= false;
  else if(msg->data == "resume")
    g_is_base_running= true;
  else
    ROS_INFO_STREAM("invalid state");
}

void guidance_distance_callback(const sensor_msgs::LaserScan& g_oa)
{
  ROS_INFO("frame_id: %s stamp: %d\n", g_oa.header.frame_id.c_str(),
           g_oa.header.stamp.sec);
  ROS_INFO("obstacle distance: [%f %f %f %f %f]\n", g_oa.ranges[0],
           g_oa.ranges[1], g_oa.ranges[2], g_oa.ranges[3], g_oa.ranges[4]);
  g_height= g_oa.ranges[0];
}
