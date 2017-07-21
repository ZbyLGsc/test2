#include "rm_challenge_vision.h"
#include "AprilTags/QRCode.h"
#define M100_CAMERA 1
#define VIDEO_STREAM 2
//#define CURRENT_IMAGE_SOURCE VIDEO_STREAM
#define CURRENT_IMAGE_SOURCE M100_CAMERA
#define VISABILITY true
#define QRCODE_VISABLE false

/**global publisher*/
ros::Publisher vision_pillar_pub;
ros::Publisher vision_line_pub;
ros::Publisher vision_base_pub;
image_transport::Publisher vision_image_pub;
/**global subscriber*/
ros::Subscriber color_change_sub;
ros::Subscriber pillar_change_sub;
ros::Subscriber line_change_sub;
/**color and task flag*/
RMChallengeVision::COLOR_TYPE g_color= RMChallengeVision::RED;
bool g_is_pillar_running= true;
bool g_is_line_running= true;

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

void colorChangeCallback(const std_msgs::String::ConstPtr &msg);
void pillarChangeCallback(const std_msgs::String::ConstPtr &msg);
void lineChangeCallback(const std_msgs::String::ConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rm_challenge_camera_node");
  ros::NodeHandle node;

  vision_pillar_pub= node.advertise<std_msgs::String>("tpp/pillar", 1);
  vision_line_pub= node.advertise<std_msgs::String>("tpp/yellow_line", 1);
  vision_base_pub= node.advertise<std_msgs::String>("tpp/base", 1);

  color_change_sub= node.subscribe("/tpp/color_change", 1, colorChangeCallback);
  pillar_change_sub=
      node.subscribe("/tpp/pillar_change", 1, pillarChangeCallback);
  line_change_sub= node.subscribe("/tpp/line_change", 1, lineChangeCallback);

  image_transport::ImageTransport image_transport(node);
  vision_image_pub= image_transport.advertise("m100/image", 1);

  // ros::Timer cap_timer =
  //     node.createTimer( ros::Duration( 1.0 / 100.0 ),
  //     cap_timer_callback );
  // ros::Timer pillar_timer =
  //     node.createTimer( ros::Duration( 1.0 / 100.0 ),
  //     pillar_timer_callback );
  // ros::Timer line_timer =
  //     node.createTimer( ros::Duration( 1.0 / 100.0 ),
  //     line_timer_callback );
  // ros::Timer base_timer =
  //     node.createTimer( ros::Duration( 1.0 / 100.0 ),
  //     base_timer_callback );
  // cv::VideoCapture cap(
  // "/home/zby/uav_slam_ws/src/rm_uav/res/color_ball4.avi"
  // );
  cv::VideoCapture g_cap;
#if CURRENT_IMAGE_SOURCE == VIDEO_STREAM
  // g_cap.open("/home/ubuntu/rosbag/Tttt.avi");
  g_cap.open("/home/zby/ros_bags/7.19/TTT.avi");
#else
  g_cap.open(0);
#endif

  if(!g_cap.isOpened())
  {
    ROS_INFO("camera not open");
    return -1;
  }
  RMChallengeVision vision;
  vision.setVisability(VISABILITY);

  QRCode qr_code;
  qr_code.setVisability(QRCODE_VISABLE);
  qr_code.setup();

  Mat frame, image_gray;
  sensor_msgs::ImagePtr image_ptr;

  /*get first pillar's color*/
  ROS_INFO_STREAM("Please give the first pillar's color(r/b):");
  char first_pillar_color= getchar();
  if(first_pillar_color == 'r')
  {
    g_color= RMChallengeVision::RED;
  }
  else if(first_pillar_color == 'b')
  {
    g_color= RMChallengeVision::BLUE;
  }
  else
  {
    ROS_INFO_STREAM("not a available color!");
    return -2;
  }

  while(ros::ok())
  {
    ROS_INFO_STREAM("loop :"
                    << "\n");

#if CURRENT_IMAGE_SOURCE == VIDEO_STREAM
    /*get new frame*/
    if(g_cap.get(CV_CAP_PROP_POS_FRAMES) >
       g_cap.get(CV_CAP_PROP_FRAME_COUNT) / 2)
    {
      g_cap.set(CV_CAP_PROP_POS_FRAMES, 10);
    }
#endif

    g_cap >> frame;
    if(frame.empty())
      continue;

    /* publish this frame to ROS topic*/
    image_ptr=
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    vision_image_pub.publish(image_ptr);

    /*std_msg of string published to uav*/
    std::stringstream ss;

    /*test detect pillar circle and triangles*/
    if(g_is_pillar_running)
    {
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
        // calculate height and pos_error
        // arc_height=
        //     vision.imageToHeight(pillar_result.arc_radius, 200.0);
        // arc_err_x= vision.imageToRealDistance(
        //     pillar_result.arc_radius, pillar_result.arc_center.x,
        //     200.0);
        // arc_err_y= vision.imageToRealDistance(
        //     pillar_result.arc_radius, pillar_result.arc_center.y,
        //     200.0);

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

      // cv::waitKey(1);
      // ros::spinOnce();
      // continue;
    }

    /*test detect yellow line*/
    if(g_is_line_running)
    {
      //    ROS_INFO_STREAM("detect line");
      float distance_x, distance_y, line_vector_x, line_vector_y;
      bool is_T_found= vision.detectLineWithT(frame, distance_x, distance_y,
                                              line_vector_x, line_vector_y);
      if(is_T_found)
        ROS_INFO_STREAM("T");
      else
      {
        ROS_INFO_STREAM("distance:" << distance_x << " " << distance_y);
        ROS_INFO_STREAM("line direction:" << line_vector_x << " "
                                          << line_vector_y);
      }
      // publish result
      ss.str("");
      std_msgs::String line_msg;
      ss << is_T_found << " " << distance_x << " " << distance_y << " "
         << line_vector_x << " " << line_vector_y;
      line_msg.data= ss.str();
      vision_line_pub.publish(line_msg);
    }

    /*detect QRCode*/
    bool base_found;
    if(qr_code.getBasePosition(frame, 2.4))
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
    ss.str("");
    std_msgs::String base_msg;
    ss << base_found << " " << qr_code.getBaseX() << " " << qr_code.getBaseY();
    base_msg.data= ss.str();
    vision_base_pub.publish(base_msg);

    ros::spinOnce();

    cv::waitKey(1);
  }

  return 1;
}

void colorChangeCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO_STREAM("receive color change info");
  g_color= g_color == RMChallengeVision::RED ? RMChallengeVision::BLUE :
                                               RMChallengeVision::RED;
}

void pillarChangeCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO_STREAM("receive pillar change info");
  g_is_pillar_running= !g_is_pillar_running;
}

void lineChangeCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO_STREAM("receive line change info");
  g_is_line_running= !g_is_line_running;
}