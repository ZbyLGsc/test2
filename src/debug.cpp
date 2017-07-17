#include <ros/assert.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <vector>
#include <math.h>
#include "rm_challenge_fsm.h"

#define ARRAY_SIZE 50
float pos_errors[ARRAY_SIZE + 2];
int array_counter= 0;
bool should_stop= false;

void velocityCallback(const geometry_msgs::Vector3Stamped &vel)
{
  if(vel.header.frame_id != "arc error")
    return;
  float error= sqrt(pow(vel.vector.x, 2) + pow(vel.vector.y, 2));
  ROS_INFO_STREAM("error:" << error);
  /*fill the array with number*/
  if(error < 0.0001)
    return;
  if(array_counter < ARRAY_SIZE)
  {
    pos_errors[array_counter + 1]= error;
    array_counter++;
  }
  /*insert this error into full array from small to big*/
  else
  {
    /*find the array id which this error should be put*/
    for(int i= 0; i < ARRAY_SIZE; i++)
    {
      if(error > pos_errors[i] && error < pos_errors[i + 1])
      {
        /*move the array backward*/
        for(int j= ARRAY_SIZE - 2; j > i + 1; j--)
        {
          pos_errors[j]= pos_errors[j - 1];
        }
        pos_errors[i + 1]= error;
      }
    }
  }

  /*print out result*/
  ROS_INFO_STREAM("result:");
  for(int i= 1; i < ARRAY_SIZE - 1; i++)
  {
    ROS_INFO_STREAM("error " << i << " is:" << pos_errors[i]);
  }
}

void velocityCallback2(const geometry_msgs::Vector3Stamped vel)
{
  if(vel.header.frame_id == "arc error")
  {
    float error= sqrt(pow(vel.vector.x, 2) + pow(vel.vector.y, 2));
    float h_error= fabs(vel.vector.z - PA_LAND_HEIGHT_FINAL);
    // ROS_INFO_STREAM("error:" << error<<" height:"<<h_error);
    //&&h_error < PA_LAND_HEIGHT_THRESHOLD_FINAL
    if(error > 0.0005 && error < 0.1)
    {
      ROS_INFO_STREAM("!!!!!!!!!error:" << error
                                        << " height:" << h_error);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug");
  ros::NodeHandle node;
  ros::Subscriber velocity_sub=
      node.subscribe("/m100/velocity", 1, velocityCallback2);

  float min= -1, max= 1000;
  pos_errors[0]= min;
  pos_errors[ARRAY_SIZE - 1]= max;

  ROS_INFO_STREAM("begin to run:");
  while(ros::ok() && !should_stop)
  {
    ros::spinOnce();
  }

  return 1;
}