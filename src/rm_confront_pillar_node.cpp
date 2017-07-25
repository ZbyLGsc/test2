#include "rm_challenge_vision.h"
#define M100_CAMERA 1
#define VIDEO_STREAM 2
//#define CURRENT_IMAGE_SOURCE VIDEO_STREAM
#define CURRENT_IMAGE_SOURCE M100_CAMERA
#define VISABILITY true

/**global publisher*/
ros::Publisher vision_pillar_pub;
image_transport::Publisher vision_image_pub;
/**color and task flag*/
RMChallengeVision::COLOR_TYPE g_color= RMChallengeVision::RED;
bool g_is_pillar_running= true;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rm_challenge_camera_node");
  ros::NodeHandle node;

  vision_pillar_pub= node.advertise<std_msgs::String>("tpp/pillar", 1);

  image_transport::ImageTransport image_transport(node);
  vision_image_pub= image_transport.advertise("m100/image", 1);

  cv::VideoCapture g_cap;
  cv::VideoWriter g_writer;
#if CURRENT_IMAGE_SOURCE == VIDEO_STREAM
   g_cap.open("/home/ubuntu/rosbag/3334.avi");
  //g_cap.open("/home/zby/ros_bags/7.22/start1.avi");
  //g_cap.set(CV_CAP_PROP_POS_FRAMES, g_cap.get(CV_CAP_PROP_FRAME_COUNT) / 2);
  g_cap.set(CV_CAP_PROP_POS_FRAMES,100);
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

  Mat frame, image_gray;
  sensor_msgs::ImagePtr image_ptr;

  /*get first pillar's color from user*/
  ROS_INFO_STREAM("Please give the first pillar's color(r/b):");
  char first_pillar_color;
  std::cin >> first_pillar_color;
  // cout<<argv[1]<<endl;
  // first_pillar_color = argv[1][0];
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

  /*get want to record video or no  t*/
  ROS_INFO_STREAM("Want to record video or not?(y/n):");
  char want_record_video;
  std::cin >> want_record_video;
  // want_record_video = argv[1][1];
  if(want_record_video == 'y')
  {
    std::string file_name;
    ROS_INFO_STREAM("Begin record video to file,please give a file name");
    std::cin >> file_name;
    // file_name= "/home/zby/ros_bags/" + file_name + ".avi";
    file_name= "/home/ubuntu/rosbag/" + file_name + ".avi";
    g_writer.open(file_name, CV_FOURCC('P', 'I', 'M', '1'), 30,
                  cv::Size(640, 480));
  }
  else if(want_record_video == 'n')
  {
    ROS_INFO_STREAM("Will not record video ");
  }
  else
  {
    ROS_INFO_STREAM("not a available selection!");
    return -2;
  }
  while(ros::ok())
  {
    ROS_INFO_STREAM("loop :"
                    << "\n");

#if CURRENT_IMAGE_SOURCE == VIDEO_STREAM
    /*get new frame*/
    if(g_cap.get(CV_CAP_PROP_POS_FRAMES) >
       g_cap.get(CV_CAP_PROP_FRAME_COUNT) -1)
    {
      //g_cap.set(CV_CAP_PROP_POS_FRAMES, g_cap.get(CV_CAP_PROP_FRAME_COUNT) / 2);
	  g_cap.set(CV_CAP_PROP_POS_FRAMES,100);
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
      /*show current color*/
      std::string color;
      if(g_color==RMChallengeVision::RED)
        color="Red";
      else if(g_color==RMChallengeVision::BLUE)
        color="Blue";
      ROS_INFO_STREAM("Color is: "<<color);
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

      // cv::waitKey(1);
      // ros::spinOnce();
      // continue;
    }

    /*record image to file*/
    if(want_record_video == 'y')
    {
      g_writer.write(frame);
    }

    ros::spinOnce();
    cv::waitKey(1);
  }
  g_writer.release();
  return 1;
}


