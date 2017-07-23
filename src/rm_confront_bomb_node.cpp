#define ZBY_PC 1
#define MANIFOLD 2
#define CURRENT_COMPUTER ZBY_PC
// #define CURRENT_COMPUTER MANIFOLD

/**RC channel define*/
#define RC_F_UP 0
#define RC_F_DOWN 1
#define RC_P_UP 2
#define RC_P_DOWN 3
#define RC_A_UP 4
#define RC_A_DOWN 5

// ros
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include "std_msgs/String.h"

// boost asio
#include <boost/asio.hpp>
#include <boost/bind.hpp>
using namespace boost::asio;

#if CURRENT_COMPUTER == MANIFOLD
dji_sdk::RCChannels g_rc_channels;
void rc_channels_callback(const dji_sdk::RCChannels rc_channels);
#endif
void uav_state_callback(const std_msgs::UInt8::ConstPtr &msg);
void vision_base_callback(const std_msgs::String::ConstPtr &msg);
/**timer callback, control uav in a finite state machine*/
void timer_callback(const ros::TimerEvent &evt);

/**
*global variables
*/
int g_RC_channel= RC_P_UP;
int g_current_channel= -1;
/**serial port*/
boost::asio::serial_port *g_serial_port;
boost::system::error_code m_err_code;
boost::asio::io_service m_io_service;

/**
*global functions
*/
void informGraspperChange(std::string state);
void initilizeSerialPort();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rm_uav_challenge");
  ros::NodeHandle node;
/*subscriber from dji node*/
#if CURRENT_COMPUTER == MANIFOLD
  ros::Subscriber rc_channels_sub=
      node.subscribe("dji_sdk/rc_channels", 1, rc_channels_callback);
#endif
  ros::Subscriber uav_state_sub=
      node.subscribe("/dji_sdk/flight_status", 1, uav_state_callback);

  ros::Subscriber vision_base_sub=
      node.subscribe("tpp/base", 1, vision_base_callback);

  ros::Timer timer= node.createTimer(ros::Duration(1.0 / 50.0), timer_callback);

  initilizeSerialPort();

  ros::spin();
  return 0;
}

#if CURRENT_COMPUTER == MANIFOLD
void rc_channels_callback(const dji_sdk::RCChannels rc_channels)
{
  /*receive rc channel here and set flags*/
  g_rc_channels= rc_channels;
  if(fabs(rc_channels.mode - 8000) < 0.000001)
  {
    if(fabs(rc_channels.gear + 10000) < 0.000001)
    {
      g_RC_channel= RC_F_UP;
      ROS_INFO_STREAM("RC channel is: F up");
    }
    else if(fabs(rc_channels.gear + 4545) < 0.000001)
    {
      g_RC_channel= RC_F_DOWN;
      ROS_INFO_STREAM("RC channel is: F down");
    }
  }
  else if(fabs(rc_channels.mode + 8000) < 0.000001)
  {
    if(fabs(rc_channels.gear + 10000) < 0.000001)
    {
      g_RC_channel= RC_P_UP;
      ROS_INFO_STREAM("RC channel is: P up");
    }
    else if(fabs(rc_channels.gear + 4545) < 0.000001)
    {
      g_RC_channel= RC_P_DOWN;
      ROS_INFO_STREAM("RC channel is: P down");
    }
  }
  else if(fabs(rc_channels.mode - 0.0) < 0.000001)
  {
    if(fabs(rc_channels.gear + 10000) < 0.000001)
    {
      g_RC_channel= RC_A_UP;
      ROS_INFO_STREAM("RC channel is: A up");
    }
    else if(fabs(rc_channels.gear + 4545) < 0.000001)
    {
      g_RC_channel= RC_A_DOWN;
      ROS_INFO_STREAM("RC channel is: A down");
    }
  }
}
#endif

void uav_state_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  int flight_status= msg->data;
}

void vision_base_callback(const std_msgs::String::ConstPtr &msg)
{
  float pos[2];
  bool base_found;
  std::stringstream ss(msg->data.c_str());
  ss >> base_found >> pos[0] >> pos[1];
}

void timer_callback(const ros::TimerEvent &evt)
{
  ROS_INFO_STREAM("loop:");
  /*do different task according to mode*/
  switch(g_RC_channel)
  {
    case RC_P_UP:
    {
      /*release grabber*/
      informGraspperChange("open");
      break;
    }
    case RC_P_DOWN:
    {
      /*close grabber*/
      informGraspperChange("close");
      break;
    }
    case RC_F_UP:
    {
      break;
    }
    case RC_F_DOWN:
    {
      break;
    }
    case RC_A_UP:
    {
      break;
    }
    case RC_A_DOWN:
    {
      break;
    }
  }
}

void informGraspperChange(std::string state)
{
  if(g_RC_channel == g_current_channel)
    return;
  else if(state == "open")
  {
    for(int i= 0; i < 10; i++)
      boost::asio::write(*g_serial_port, boost::asio::buffer("b"), m_err_code);
    g_current_channel= g_RC_channel;
    ROS_INFO_STREAM("open");
  }
  else if(state == "close")
  {
    for(int i= 0; i < 10; i++)
      boost::asio::write(*g_serial_port, boost::asio::buffer("a"), m_err_code);
    g_current_channel= g_RC_channel;
    ROS_INFO_STREAM("close");
  }
}

void initilizeSerialPort()
{
  g_serial_port= new boost::asio::serial_port(m_io_service);
  g_serial_port->open("/dev/ttyTHS0", m_err_code);
  g_serial_port->set_option(serial_port::baud_rate(9600), m_err_code);
  g_serial_port->set_option(
      serial_port::flow_control(serial_port::flow_control::none), m_err_code);
  g_serial_port->set_option(serial_port::parity(serial_port::parity::none),
                            m_err_code);
  g_serial_port->set_option(serial_port::stop_bits(serial_port::stop_bits::one),
                            m_err_code);
  g_serial_port->set_option(serial_port::character_size(8), m_err_code);
}