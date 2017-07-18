#include "rm_challenge_vision.h"
#include "AprilTags/QRCode.h"
#define M100_CAMERA 1
#define VIDEO_STREAM 2
//  #define CURRENT_IMAGE_SOURCE VIDEO_STREAM
#define CURRENT_IMAGE_SOURCE M100_CAMERA
#define VISABILITY false

/**global publisher*/
ros::Publisher vision_pillar_pub;
ros::Publisher vision_line_pub;
ros::Publisher vision_base_pub;
image_transport::Publisher vision_image_pub;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rm_challenge_camera_node");
  ros::NodeHandle node;

  vision_pillar_pub=
      node.advertise<std_msgs::String>("tpp/pillar", 1);
  vision_line_pub=
      node.advertise<std_msgs::String>("tpp/yellow_line", 1);
  vision_base_pub= node.advertise<std_msgs::String>("tpp/base", 1);

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
  g_cap.open("/home/jachinshen/视频/arc2.avi");
#else
  g_cap.open(1);
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

  QRCode qr_code;
  qr_code.setVisability(false);
  qr_code.setup();

  while(ros::ok())
  {
    ROS_INFO_STREAM("loop :"
                    << "\n");

#if CURRENT_IMAGE_SOURCE == VIDEO_STREAM
    /*get new frame*/
    if(g_cap.get(CV_CAP_PROP_POS_FRAMES) >
       g_cap.get(CV_CAP_PROP_FRAME_COUNT) - 1)
    {
      g_cap.set(CV_CAP_PROP_POS_FRAMES,
                g_cap.get(CV_CAP_PROP_FRAME_COUNT) / 2);
    }
#endif

    g_cap >> frame;
    if(frame.empty())
      continue;

    /* publish this frame to ROS topic*/
    image_ptr= cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame)
                   .toImageMsg();
    vision_image_pub.publish(image_ptr);

    /*std_msg of string published to uav*/
    std::stringstream ss;

    /*test detect pillar circle and triangles*/
    //    ROS_INFO_STREAM("detect pillar");
    RMChallengeVision::PILLAR_RESULT pillar_result;
    float pos_err_x= 0, pos_err_y= 0, height= 0;
    float arc_err_x= 1, arc_err_y= 1, arc_height= 2;
    vision.detectPillar(frame, pillar_result);
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
    ss << pillar_result.triangle[0] << " "
       << pillar_result.triangle[1] << " "
       << pillar_result.triangle[2] << " "
       << pillar_result.triangle[3] << " "
       << pillar_result.circle_found << " " << pos_err_x << " "
       << pos_err_y << " " << height << " " << arc_err_x << " "
       << arc_err_y << " " << pillar_result.arc_found;
    pillar_msg.data= ss.str();
    vision_pillar_pub.publish(pillar_msg);

    /*test detect yellow line*/
    //    ROS_INFO_STREAM("detect line");
    float distance_x, distance_y, line_vector_x, line_vector_y;
    if(vision.detectLineWithT(frame, distance_x, distance_y,
                              line_vector_x, line_vector_y))
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
    ss << distance_x << " " << distance_y << " " << line_vector_x
       << " " << line_vector_y;
    line_msg.data= ss.str();
    vision_line_pub.publish(line_msg);

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
    ss << base_found << " " << qr_code.getBaseX() << " "
       << qr_code.getBaseY();
    base_msg.data= ss.str();
    vision_base_pub.publish(base_msg);

    cv::waitKey(1);
  }

  return 1;
}

// void cap_timer_callback( const ros::TimerEvent &evt )
// {
//     ROS_INFO_STREAM( "process time" << g_processed_time );
//     if ( g_processed_time > 0 )
//         return;

//     if ( g_cap.get( CV_CAP_PROP_POS_FRAMES ) >
//          g_cap.get( CV_CAP_PROP_FRAME_COUNT ) - 2 )
//     {
//         g_cap.set( CV_CAP_PROP_POS_FRAMES, 0 );
//     }
//     Mat frame;
//     g_cap >> frame;
//     ROS_INFO_STREAM( "loop :"
//                      << "\n" );
//     g_pillar_image = frame.clone();
//     g_line_image = frame.clone();
//     g_base_image = frame.clone();
//     g_processed_time = 3;
// }
// void pillar_timer_callback( const ros::TimerEvent &evt )
// {
//     if ( g_processed_time <= 0 )
//         return;

//     RMChallengeVision::PILLAR_RESULT pillar_result;
//     float pos_err_x = 0, pos_err_y = 0, height = 0;
//     vision.detectPillar( g_pillar_image, pillar_result );
//     if ( pillar_result.circle_found )
//     {
//         // calculate height and pos_error
//         height = vision.imageToHeight( pillar_result.radius, 250.0
//         );
//         pos_err_x = vision.imageToRealDistance(
//             pillar_result.radius, pillar_result.circle_center.x,
//             250.0 );
//         pos_err_y = vision.imageToRealDistance(
//             pillar_result.radius, pillar_result.circle_center.y,
//             250.0 );
//     }
//     // publish result to uav
//     std_msgs::String pillar_msg;
//     std::stringstream ss;
//     ss << pillar_result.triangle[0] << " " <<
//     pillar_result.triangle[1] << " "
//        << pillar_result.triangle[2] << " " <<
//        pillar_result.triangle[3] << " "
//        << pillar_result.circle_found << " " << pos_err_x << " " <<
//        pos_err_y << "
//        "
//        << height;
//     pillar_msg.data = ss.str();
//     vision_pillar_pub.publish( pillar_msg );

//     g_processed_time--;
// }
// void line_timer_callback( const ros::TimerEvent &evt )
// {
//     if ( g_processed_time <= 0 )
//         return;

//     float distance_x, distance_y, line_vector_x, line_vector_y;
//     vision.detectLine( g_line_image, distance_x, distance_y,
//     line_vector_x,
//                         line_vector_y );
//     ROS_INFO_STREAM( "line :" << distance_x << " " << distance_y <<
//     line_vector_x
//                               << " " << line_vector_y );
//     // publish result
//     std::stringstream ss;
//     std_msgs::String line_msg;
//     ss << distance_x << " " << distance_y << " " << line_vector_x
//     << " "
//        << line_vector_y;
//     line_msg.data = ss.str();
//     vision_line_pub.publish( line_msg );

//     g_processed_time--;
// }
// void base_timer_callback( const ros::TimerEvent &evt )
// {
//     if ( g_processed_time <= 0 )
//         return;

//     g_processed_time--;
// }
