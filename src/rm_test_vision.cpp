#include "rm_challenge_vision.h"

/**subscribe image from m100*/
image_transport::Subscriber m100_image_sub;
cv::Mat g_m100_image;
RMChallengeVision vision;
void m100ImageCallback(const sensor_msgs::Image::ConstPtr msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rm_test_vision");
  ros::NodeHandle node;

  image_transport::ImageTransport image_transport(node);
  m100_image_sub=
      image_transport.subscribe("/m100/image", 1, m100ImageCallback);

  cv::namedWindow("m100_image", 1);
  vision.setVisability(true);

  /*write image to local file*/

  cv::VideoWriter writer;
  writer.open("/home/ubuntu/ros_bags/arc1.avi",
              CV_FOURCC('P', 'I', 'M', '1'), 30, cv::Size(640, 480));
  ROS_INFO_STREAM("begin main loop:");
  while(ros::ok())
  {
    ros::spinOnce();
    if(!g_m100_image.empty())
    {
      cv::imshow("m100_image", g_m100_image);
      writer.write(g_m100_image);

      // float distance_x, distance_y, line_vector_x, line_vector_y;
      // vision.detectLineWithT(g_m100_image, distance_x, distance_y,
      //                        line_vector_x, line_vector_y);
      /*test find pillar*/
      RMChallengeVision::PILLAR_RESULT pillar_result;
      vision.detectPillar(g_m100_image, pillar_result);
    }
    cv::waitKey(1);
  }

  cv::destroyAllWindows();
  writer.release();
  return 0;
}

void m100ImageCallback(const sensor_msgs::Image::ConstPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  g_m100_image= cv_ptr->image;
  ROS_INFO_STREAM("image arrive");
}
