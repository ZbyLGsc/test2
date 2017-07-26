#include "rm_challenge_vision.h"
#define M100_CAMERA 1
#define VIDEO_STREAM 2
//#define CURRENT_IMAGE_SOURCE VIDEO_STREAM
#define CURRENT_IMAGE_SOURCE M100_CAMERA
#define VISABILITY false

/**global publisher*/
ros::Publisher vision_pillar_pub;
image_transport::Publisher g_image_pub;
ros::Subscriber pillar_change_sub;
/**color and task flag*/
RMChallengeVision::COLOR_TYPE g_color= RMChallengeVision::RED;
bool g_is_pillar_running= true;
void pillarChangeCallback(const std_msgs::String::ConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rm_challenge_camera_node");
  ros::NodeHandle node;

  vision_pillar_pub= node.advertise<std_msgs::String>("tpp/pillar", 1);
  pillar_change_sub=
      node.subscribe("/tpp/pillar_task", 1, pillarChangeCallback);

  /*initialize camera and publish to pillar and base*/
  image_transport::ImageTransport image_transport(node);
  g_image_pub= image_transport.advertise("m100/image", 1);
  Mat frame, image_gray;
  sensor_msgs::ImagePtr image_ptr;

  cv::VideoCapture g_cap;
#if CURRENT_IMAGE_SOURCE == VIDEO_STREAM
  g_cap.open("/home/ubuntu/rosbag/test_confront.avi");
  // g_cap.open("/home/zby/ros_bags/7.22/start1.avi");
  // g_cap.set(CV_CAP_PROP_POS_FRAMES, g_cap.get(CV_CAP_PROP_FRAME_COUNT) / 2);
  g_cap.set(CV_CAP_PROP_POS_FRAMES, 110);
#else
  g_cap.open(0);
#endif
  if(!g_cap.isOpened())
  {
    return -1;
  }

  RMChallengeVision vision;
  vision.setVisability(VISABILITY);

  std::stringstream ss;

  /*main loop*/
  while(ros::ok())
  {
    ROS_INFO_STREAM("loop :"
                    << "\n");
    ros::spinOnce();
#if CURRENT_IMAGE_SOURCE == VIDEO_STREAM
    /*get new frame*/
    if(g_cap.get(CV_CAP_PROP_POS_FRAMES) >
       g_cap.get(CV_CAP_PROP_FRAME_COUNT) - 1)
    {
      // g_cap.set(CV_CAP_PROP_POS_FRAMES, g_cap.get(CV_CAP_PROP_FRAME_COUNT) /
      // 2);
      g_cap.set(CV_CAP_PROP_POS_FRAMES, 100);
    }
#endif

    g_cap >> frame;
    if(frame.empty())
      continue;

    ss.str("");
    /* publish this frame to ROS topic*/
    image_ptr=
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    g_image_pub.publish(image_ptr);
    /*std_msg of string published to uav*/

    /*test detect pillar circle and triangles*/
    if(g_is_pillar_running)
    {

      if(VISABILITY)
        cv::imshow("m100/image", frame);
      cv::waitKey(1);
      /*show current color*/
      std::string color;
      if(g_color == RMChallengeVision::RED)
        color= "Red";
      else if(g_color == RMChallengeVision::BLUE)
        color= "Blue";
      ROS_INFO_STREAM("Color is: " << color);
      //    ROS_INFO_STREAM("detect pillar");
      RMChallengeVision::PILLAR_RESULT pillar_result;
      float pos_err_x= 0, pos_err_y= 0, height= 0;
      float arc_err_x= 1, arc_err_y= 1, arc_height= 2;
      vision.detectPillar(frame, g_color, pillar_result);
      if(pillar_result.circle_found)
      {
        // calculate height and pos_error
        height= vision.imageToHeight(pillar_result.radius, 250.0);
        pos_err_x= vision.imageToRealDistance(
            pillar_result.radius, pillar_result.circle_center.x, 250.0);
        pos_err_y= vision.imageToRealDistance(
            pillar_result.radius, pillar_result.circle_center.y, 250.0);
      }
      if(pillar_result.arc_found)
      {
        /*send image pixel error to uav*/
        arc_err_x= pillar_result.arc_center.x;
        arc_err_y= pillar_result.arc_center.y;
      }
      // publish result to uav
      std_msgs::String pillar_msg;
      ss << pillar_result.triangle[0] << " " << pillar_result.triangle[1] << " "
         << pillar_result.triangle[2] << " " << pillar_result.triangle[3] << " "
         << pillar_result.circle_found << " " << pos_err_x << " " << pos_err_y
         << " " << height << " " << arc_err_x << " " << arc_err_y << " "
         << pillar_result.arc_found;
      pillar_msg.data= ss.str();
      vision_pillar_pub.publish(pillar_msg);
    }
  }
  return 1;
}

void pillarChangeCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO_STREAM("receive pillar change info");
  if(msg->data == "open")
    g_is_pillar_running= true;
  else if(msg->data == "close")
    g_is_pillar_running= false;
  else
    ROS_INFO_STREAM("invalid state");
}
