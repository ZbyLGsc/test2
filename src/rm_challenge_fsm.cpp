#include "rm_challenge_fsm.h"

RMChallengeFSM::~RMChallengeFSM()
{
  delete m_serial_port;
#if CURRENT_COMPUTER == MANIFOLD
  m_drone->release_sdk_permission_control();
  delete m_drone;
#endif
}
void RMChallengeFSM::run()
{
  // ROS_INFO_STREAM("running: state is:" << m_state);
  if(m_state == TAKE_OFF)  //
  {
    // send take off command to uav until state change
    if(!isTakingoff())
    {
      /* send take off command to uav*/
      closeGraspper();
      droneTakeoff();
      updateTakeoffTime();
      ros::Duration(1.0).sleep();
    }
    else
    {
      if(isTakingoff() && !isTakeoffTimeout())
      {
        /* wait for time out */
        ros::Duration(0.5).sleep();
      }
      else if(isTakeoffTimeout())
      {
        /* state change to GO_UP*/
        transferToTask(GO_UP);
      }
    }
  }
  else if(m_state == GO_UP)  //
  {
    /* not reach goal height */
    if(!closeToGoalHeight())
    {
      /* uav go up*/
      droneGoUp();
    }
    else if(closeToGoalHeight())
    {
      /* change state to setpoint*/
      transferToTask(GO_TO_SETPOINT);
    }
  }
  else if(m_state == GO_TO_SETPOINT)  //
  {
    if(!farFromTakeoffPoint())
    {
      droneGoToSetPoint();
    }
    else if(discoverLandPoint())
    {
      transferToTask(GO_TO_LAND_POINT);
    }
    else if(discoverYellowLine())
    {
      transferToTask(TRACK_LINE);
    }
    else if(!discoverLandPoint() && !discoverYellowLine() &&
            !closeToSetPoint())
    {
      droneGoToSetPoint();
    }
    else
    {
      transferToTask(IDLE);
    }
  }
  else if(m_state == TRACK_LINE)
  {
    if(!discoverLandPoint())
    {
      /* track detectLine */
      droneTrackLine();
    }
    else
    {
      /* state change to land */
      transferToTask(GO_TO_LAND_POINT);
    }
  }
  else if(m_state == GO_TO_LAND_POINT)
  {
    if(!readyToLand())
    {
      /* go to land point */
      dronePrepareToLand();
    }
    else
    {
      /* land*/
      droneHover();
      droneDropDown();
      transferToTask(LAND);
    }
  }
  else if(m_state == LAND)
  {
    if(!isOnLand())
    {
      /* continue to land */
      openGraspper();
      droneLand();
    }
    else if(isOnLand())
    {
      transferToTask(CONTROL_GRASPPER);
    }
  }
  else if(m_state == CONTROL_GRASPPER)  //
  {
    if(!finishGraspperTask())
    {
      /* continue graspper control */
      controlGraspper();
    }
    else
    {
      /* state change to take off*/
      closeGraspper();
      transferToTask(TAKE_OFF);
    }
  }
  else if(m_state == IDLE)  //
  {
    /* code */
    if(discoverLandPoint())
    {
      transferToTask(GO_TO_LAND_POINT);
    }
    else if(discoverYellowLine())
    {
      /* track detectLine*/
      transferToTask(TRACK_LINE);
    }
    else
    {
      // do nothing, wait
      droneHover();
      ros::Duration(0.5).sleep();
    }
  }
}
void RMChallengeFSM::resetAllState()
{
  ros::Duration(1.0).sleep();
  m_state= TAKE_OFF;
  m_uav_state= UAV_LAND;
  m_current_position_from_guidance[0]= 0.0;
  m_current_position_from_guidance[1]= 0.0;
  m_prepare_to_land_type= PREPARE_AT_HIGH;
  m_graspper_control_time= 0;
  m_current_takeoff_point_id= 0;
  m_land_counter= 0;
#if CURRENT_COMPUTER == MANIFOLD
  m_drone->request_sdk_permission_control();
#endif
}
void RMChallengeFSM::initialize(ros::NodeHandle &node_handle)
{
  /*initialize serial port*/
  m_serial_port= new boost::asio::serial_port(m_io_service);
  m_serial_port->open("/dev/ttyTHS0", m_err_code);
  m_serial_port->set_option(serial_port::baud_rate(9600), m_err_code);
  m_serial_port->set_option(
      serial_port::flow_control(serial_port::flow_control::none),
      m_err_code);
  m_serial_port->set_option(
      serial_port::parity(serial_port::parity::none), m_err_code);
  m_serial_port->set_option(
      serial_port::stop_bits(serial_port::stop_bits::one),
      m_err_code);
  m_serial_port->set_option(serial_port::character_size(8),
                            m_err_code);

/*initialize dji sdk*/
#if CURRENT_COMPUTER == MANIFOLD
  m_drone= new DJIDrone(node_handle);
#endif

  /*initialize publisher*/
  m_position_pub=
      node_handle.advertise<geometry_msgs::Vector3Stamped>(
          "/m100/position", 1);
  m_velocity_pub=
      node_handle.advertise<geometry_msgs::Vector3Stamped>(
          "/m100/velocity", 1);

  /*initialize parameter*/
  for(int i= 0; i < TAKEOFF_POINT_NUMBER; ++i)
  {
    /* only set for one time */
    m_goal_height[i]= PA_FLYING_HEIGHT;
    m_takeoff_points[i][0]= 0.0;
    m_takeoff_points[i][1]= 0.0;
    m_setpoints[i][0]= 8.0;
    m_setpoints[i][1]= 0.7;
  }

  /*initialize  state*/
  resetAllState();
}
void RMChallengeFSM::transferToTask(TASK_STATE task_state)
{
  if(task_state == TAKE_OFF)
  {
    m_state= TAKE_OFF;
  }
  else if(task_state == GO_UP)
  {
    m_state= GO_UP;
  }
  else if(task_state == GO_TO_SETPOINT)
  {
    m_state= GO_TO_SETPOINT;
  }
  else if(task_state == IDLE)
  {
    m_state= IDLE;
  }
  else if(task_state == TRACK_LINE)
  {
    m_state= TRACK_LINE;
  }
  else if(task_state == LAND)
  {
    m_state= LAND;
  }
  else if(task_state == CONTROL_GRASPPER)
  {
    m_state= CONTROL_GRASPPER;
  }
  else if(task_state == GO_TO_LAND_POINT)
  {
    m_state= GO_TO_LAND_POINT;
  }
}
bool RMChallengeFSM::isTakeoffTimeout()
{
  double t= ros::Time::now().toSec() - m_takeoff_time.toSec();
  ROS_INFO_STREAM("time now is:" << ros::Time::now().toSec());
  ROS_INFO_STREAM("takeoff time is:" << m_takeoff_time.toSec());
  ROS_INFO_STREAM("Taking off time is:" << t);
  if(PA_TAKEOFF_TIME < t)
  {
    ROS_INFO_STREAM("Time is enough");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("Time is too less");
    return false;
  }
}
bool RMChallengeFSM::isTakingoff()
{
  if(m_uav_state == UAV_FLY)
  {
    ROS_INFO_STREAM("Is taking off");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("Wait to take off");
    return false;
  }
}
bool RMChallengeFSM::isOnLand()
{
  if(m_uav_state == UAV_LAND)
  {
    ROS_INFO_STREAM("on land");
    if(m_current_takeoff_point_id < TAKEOFF_POINT_NUMBER - 1)
    {
      m_current_takeoff_point_id+= 1;
    }
    return true;
  }
  else
  {
    ROS_INFO_STREAM("still not on land");
    return false;
  }
}
bool RMChallengeFSM::closeToGoalHeight()
{
  float height_error= fabs(m_current_height_from_guidance -
                           m_goal_height[m_current_takeoff_point_id]);
  if(height_error < PA_TAKEOFF_HEIGHT_THRESHOLD)
  {
    ROS_INFO_STREAM("Take off height error :" << height_error
                                              << "is "
                                                 "small");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("Take off height error:" << height_error
                                             << "is too "
                                                "large");
    return false;
  }
}
bool RMChallengeFSM::farFromTakeoffPoint()
{
  double pos_error=
      sqrt(pow(m_current_position_from_guidance[0] -
                   m_takeoff_points[m_current_takeoff_point_id][0],
               2) +
           pow(m_current_position_from_guidance[1] -
                   m_takeoff_points[m_current_takeoff_point_id][1],
               2));
  if(pos_error > PA_TAKEOFF_POSITION_ERROR)
  {
    ROS_INFO_STREAM("distance to takeoff point is:" << pos_error
                                                    << ",fa"
                                                       "r");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("distance to takeoff point is:" << pos_error
                                                    << ",too "
                                                       "close");
    return false;
  }
}
bool RMChallengeFSM::discoverTriangle()
{
  int triangle_num= m_pillar_triangle[0] + m_pillar_triangle[1] +
                    m_pillar_triangle[2] + m_pillar_triangle[3];
  if(triangle_num != 0)
  {
    // ROS_INFO_STREAM("discover triangle" << triangle_num);
    return true;
  }
  else
  {
    // ROS_INFO_STREAM("no triangle");
    return false;
  }
}
bool RMChallengeFSM::discoverLandPoint()
{
  if(m_current_takeoff_point_id == 2 ||
     m_current_takeoff_point_id == 5)
  {
    if(m_discover_base)
    {
      m_land_point_type= BASE_LAND_POINT;
      ROS_INFO_STREAM("discover base");
      return true;
    }
    else
    {
      ROS_INFO_STREAM("no base");
      return false;
    }
  }
  else
  {
    if(m_discover_pillar_circle || discoverTriangle())
    {
      m_land_point_type= PILLAR_LAND_POINT;
      ROS_INFO_STREAM("circle:" << m_discover_pillar_circle);
      return true;
    }
    else
    {
      ROS_INFO_STREAM("no circle");
      return false;
    }
  }
}
bool RMChallengeFSM::discoverYellowLine()
{
  if(fabs(m_distance_to_line[0]) > 0.0001 ||
     fabs(m_distance_to_line[1]) > 0.0001)
  {
    ROS_INFO_STREAM("discover yellow detectLine");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("no yellow detectLine found");
    return false;
  }
}
bool RMChallengeFSM::closeToSetPoint()
{
  float disp_x= m_current_position_from_guidance[0] -
                m_takeoff_points[m_current_takeoff_point_id][0];
  float disp_y= m_current_position_from_guidance[1] -
                m_takeoff_points[m_current_takeoff_point_id][1];
  double pos_error= sqrt(
      pow(disp_x - m_setpoints[m_current_takeoff_point_id][0], 2) +
      pow(disp_y - m_setpoints[m_current_takeoff_point_id][1], 2));
  if(pos_error < PA_SETPOINT_POSITION_ERROR)
  {
    ROS_INFO_STREAM("Error to setpoint is :" << pos_error << ",clos"
                                                             "e");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("Error to setpoint is :" << pos_error << ",too "
                                                             "far");
    return false;
  }
}
bool RMChallengeFSM::finishGraspperTask()
{
  if(m_graspper_control_time >= PA_GRASPPER_CONTROL_TIME)
  {
    m_graspper_control_time= 0;
    ROS_INFO_STREAM("graspper  finish");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("graspper not finish");
    return false;
  }
}
void RMChallengeFSM::droneTakeoff()
{
#if CURRENT_COMPUTER == MANIFOLD
  m_drone->takeoff();
#endif
}
void RMChallengeFSM::droneLand()
{
#if CURRENT_COMPUTER == MANIFOLD
  m_drone->landing();
#endif
}
void RMChallengeFSM::openGraspper()
{
  boost::asio::write(*m_serial_port, boost::asio::buffer("b"),
                     m_err_code);  // open graspper
  m_graspper_state= GRASPPER_OPEN;
}
void RMChallengeFSM::closeGraspper()
{
  boost::asio::write(*m_serial_port, boost::asio::buffer("a"),
                     m_err_code);  // close graspper
  m_graspper_state= GRASPPER_CLOSE;
}
void RMChallengeFSM::controlGraspper()
{
  m_graspper_control_time++;
  if(m_graspper_state == GRASPPER_CLOSE)
  {
    openGraspper();
  }
  else
  {
    closeGraspper();
  }
  ROS_INFO_STREAM("graspper state is:" << m_graspper_state);
}
void RMChallengeFSM::updateTakeoffTime()
{
  m_takeoff_time= ros::Time::now();
  ROS_INFO_STREAM("Taking off time is :" << m_takeoff_time);
}
void RMChallengeFSM::controlDroneVelocity(float x, float y, float z,
                                          float yaw)
{
#if CURRENT_COMPUTER == MANIFOLD
  m_drone->attitude_control(0x4B, x, y, z, yaw);
#endif
  ros::Duration(20 / 1000).sleep();
}
void RMChallengeFSM::droneGoUp()
{
  if(m_goal_height[m_current_takeoff_point_id] >
     m_current_height_from_guidance)
  {
    controlDroneVelocity(0.0, 0.0, PA_GO_UP_VELOCITY, 0.0);
    ROS_INFO_STREAM("go up");
  }
  else
  {
    controlDroneVelocity(0.0, 0.0, -PA_GO_UP_VELOCITY, 0.0);
    ROS_INFO_STREAM("go down");
  }
}
void RMChallengeFSM::droneGoToSetPoint()
{
  float vt_x, vt_y;
  calculateTangentialVelocity(vt_x, vt_y, VIRTUAL_LINE);
  ROS_INFO_STREAM("vtx:" << vt_x << " vt_y:" << vt_y);
  float vn_x, vn_y;
  calculateNormalVelocity(vn_x, vn_y, VIRTUAL_LINE);
  ROS_INFO_STREAM("vnx:" << vn_x << " vn_y:" << vn_y);
  controlDroneVelocity(vt_x + vn_x, vt_y + vn_y, 0.0, 0.0);

  /*publish velocity*/
  geometry_msgs::Vector3Stamped velocity;
  velocity.header.frame_id= "to_setpoint_velocity";
  velocity.header.stamp= ros::Time::now();
  velocity.vector.x= vt_x + vn_x;
  velocity.vector.y= vt_y + vn_y;
  velocity.vector.z= 0.0;
  m_velocity_pub.publish(velocity);
}
void RMChallengeFSM::droneTrackLine()
{
  float vt_x, vt_y;
  float vn_x, vn_y;
  float v_z, yaw;

  calculateTangentialVelocity(vt_x, vt_y, YELLOW_LINE);
  calculateNormalVelocity(vn_x, vn_y, YELLOW_LINE);
  calculateYawRate(yaw);
  if(fabs(m_current_height_from_guidance - PA_FLYING_HEIGHT) >
     PA_FLYING_HEIGHT_THRESHOLD)
  {
    v_z= PA_FLYING_HEIGHT > m_current_height_from_guidance ?
             PA_FLYING_Z_VELOCITY :
             -PA_FLYING_Z_VELOCITY;
  }
  ROS_INFO_STREAM("\n t:" << vt_x << "," << vt_y << "\n"
                          << "n:" << vn_x << "," << vn_y << "\n"
                          << "yaw:" << yaw << "\n"
                          << "vz:" << v_z);
  controlDroneVelocity(vt_x + vn_x, vt_y + vn_y, v_z, yaw);

  /*publish velocity*/
  geometry_msgs::Vector3Stamped velocity;
  velocity.header.frame_id= "trackline_velocity";
  velocity.header.stamp= ros::Time::now();
  velocity.vector.x= vt_x + vn_x;
  velocity.vector.y= vt_y + vn_y;
  velocity.vector.z= v_z;
  m_velocity_pub.publish(velocity);
}
void RMChallengeFSM::droneHover()
{
  controlDroneVelocity(0, 0, 0, 0);
  ros::Duration(0.5).sleep();
}

void RMChallengeFSM::droneDropDown()
{
  for(int i= 0; i < 200; i++)
    controlDroneVelocity(0.0, 0.0, -1.2, 0.0);
}

bool RMChallengeFSM::readyToLand()
{
  float land_err= sqrt(pow(m_circle_position_error[0], 2) +
                       pow(m_circle_position_error[1], 2));
  float height_error;
  if(m_land_point_type == BASE_LAND_POINT)
  {
    height_error=
        fabs(PA_LAND_HEIGHT - m_current_height_from_guidance);
    if(height_error < PA_LAND_HEIGHT_THRESHOLD &&
       land_err < PA_LAND_POSITION_THRESHOLD_LOW)
    {
      ROS_INFO_STREAM("ready to land at base," << land_err << ","
                                               << height_error);
      return true;
    }
    else
    {
      // ROS_INFO_STREAM("error too big," << land_err << ","
      //                                  << height_error);
      return false;
    }
  }
  else if(m_land_point_type == PILLAR_LAND_POINT)
  {
    height_error=
        fabs(PA_LAND_HEIGHT_FINAL - m_current_height_from_guidance);
    float pos_error= sqrt(pow(m_arc_position_error[0], 2) +
                          pow(m_arc_position_error[1], 2));
    // need output
    if(m_prepare_to_land_type == PREPARE_AT_SUPER_LOW &&
       pos_error < PA_LAND_POSITION_THRESHOLD_SUPER_LOW &&
       height_error < PA_LAND_HEIGHT_THRESHOLD_FINAL)
    {
      m_land_counter++;
      if(m_land_counter >= PA_LAND_COUNT)
      {
        m_prepare_to_land_type= PREPARE_AT_HIGH;
        m_land_counter= 0;
        ROS_INFO_STREAM("ready to land at pillar ,"
                        << pos_error << "," << height_error);
        return true;
      }
      else
      {
        ROS_INFO_STREAM("counter not enough," << land_err << ","
                                              << height_error << ","
                                              << m_land_counter);
        return false;
      }
    }
    else
    {
      // ROS_INFO_STREAM("error too big," << land_err << ","
      //                                  << height_error);
      return false;
    }
  }
}
void RMChallengeFSM::dronePrepareToLand()
{
  float vx= 0, vy= 0, vz= 0;
  std::string velocity_id;
  if(m_land_point_type == BASE_LAND_POINT)
  {
    if(fabs(m_current_height_from_guidance - PA_LAND_HEIGHT) >
       PA_BASE_HEIGHT_THRESHOLD)
    {
      vz= PA_LAND_HEIGHT > m_current_height_from_guidance ?
              PA_LAND_Z_VELOCITY :
              -PA_LAND_Z_VELOCITY;
    }
    else
    {
      vz= 0;
    }
    vx= PA_KP_BASE * m_circle_position_error[0];
    vy= PA_KP_BASE * m_circle_position_error[1];
    ROS_INFO_STREAM("landing at base v are:" << vx << "," << vy << ","
                                             << vz);
  }
  else if(m_land_point_type == PILLAR_LAND_POINT)
  {
    if(m_prepare_to_land_type == PREPARE_AT_SUPER_LOW)
    {
      navigateByArc(vx, vy, vz);
      velocity_id= "by arc";
    }
    else if(m_discover_pillar_circle)
    {
      navigateByCircle(vx, vy, vz);
      velocity_id= "by circle";
    }
    else if(discoverTriangle())
    {
      navigateByTriangle(vx, vy, vz);
      velocity_id= "by triangle";
      ROS_INFO_STREAM("navigate by triangle");
    }
    else
    {
      ROS_INFO_STREAM("Miss pillar!!!");
      velocity_id= "miss pillar";
    }
    ROS_INFO_STREAM("landing v at pillar are:" << vx << "," << vy
                                               << "," << vz);
  }
  controlDroneVelocity(vx, vy, vz, 0.0);

  /*publish velocity*/
  publishVelocity(velocity_id, vx, vy, vz);
  if(velocity_id == "by arc")
  {
    /*publish position and height error to compare*/
    publishVelocity("arc error", m_arc_position_error[0],
                    m_arc_position_error[1],
                    m_current_height_from_guidance);
  }

  // geometry_msgs::Vector3Stamped velocity;
  // velocity.header.frame_id= "landing_velocity";
  // velocity.header.stamp= ros::Time::now();
  // velocity.vector.x= vx;
  // velocity.vector.y= vy;
  // velocity.vector.z= vz;
  // m_velocity_pub.publish(velocity);
}

void RMChallengeFSM::publishVelocity(std::string id, float x, float y,
                                     float z)
{
  geometry_msgs::Vector3Stamped vector3;
  vector3.header.frame_id= id;
  vector3.header.stamp= ros::Time::now();
  vector3.vector.x= x;
  vector3.vector.y= y;
  vector3.vector.z= z;
  m_velocity_pub.publish(vector3);
}

void RMChallengeFSM::navigateByCircle(float &vx, float &vy, float &vz)
{
  ROS_INFO_STREAM("navigate by circle");
  if(m_prepare_to_land_type == PREPARE_AT_HIGH)
  {
    ROS_INFO_STREAM("navigate high");
    float land_err= sqrt(pow(m_circle_position_error[0], 2) +
                         pow(m_circle_position_error[1], 2));
    if(land_err > PA_LAND_POSITION_THRESHOLD_HIGH)
    {
      vx= PA_KP_PILLAR_HIGH * m_circle_position_error[0];
      vy= PA_KP_PILLAR_HIGH * m_circle_position_error[1];
      vz= 0;
      if(fabs(vx) < PA_V_MIN_HIGH)
        vx= fabs(vx) / (vx + 0.0001) * PA_V_MIN_HIGH;
      if(fabs(vy) < PA_V_MIN_HIGH)
        vy= fabs(vy) / (vy + 0.0001) * PA_V_MIN_HIGH;
    }
    else
    {
      vx= vy= 0.0;
      float height_error=
          PA_LAND_HEIGHT - m_current_height_from_circle;
      if(fabs(height_error) > PA_LAND_HEIGHT_THRESHOLD)
      {
        vz= fabs(height_error) / (height_error + 0.0000000001) *
            PA_LAND_Z_VELOCITY;
      }
      else
      {
        /*shift to PREPARE_AT_LOW when height and position
        error are small enough*/
        vz= 0.0;
        droneHover();
        m_prepare_to_land_type= PREPARE_AT_LOW;
      }
    }
  }
  else if(m_prepare_to_land_type == PREPARE_AT_LOW)
  {
    /*only use circle position error to adjust position*/
    ROS_INFO_STREAM("navigate at low");
    vx= PA_KP_PILLAR_LOW * m_circle_position_error[0];
    vy= PA_KP_PILLAR_LOW * m_circle_position_error[1];
    vz= 0;
    if(fabs(vx) < PA_V_MIN_LOW)
      vx= fabs(vx) / (vx + 0.0001) * PA_V_MIN_LOW;
    if(fabs(vy) < PA_V_MIN_LOW)
      vy= fabs(vy) / (vy + 0.0001) * PA_V_MIN_LOW;

    /*shift to PREPARE_AT_SUPER_LOW when position error
    is small enough */
    float land_err= sqrt(pow(m_circle_position_error[0], 2) +
                         pow(m_circle_position_error[1], 2));
    if(land_err < PA_LAND_POSITION_THRESHOLD_LOW)
    {
      m_prepare_to_land_type= PREPARE_AT_SUPER_LOW;
    }
  }
}

void RMChallengeFSM::navigateByArc(float &vx, float &vy, float &vz)
{
  ROS_INFO_STREAM("navigate at super low");
  float height_error=
      PA_LAND_HEIGHT_FINAL - m_current_height_from_guidance;
  if(fabs(height_error) > PA_LAND_HEIGHT_THRESHOLD_FINAL)
  {
    /*height error too big, use height from guidance to navigate*/
    vz= fabs(height_error) / (height_error + 0.0000000001) *
        PA_LAND_Z_VELOCITY_FINAL;
  }
  else
  {
    vz=0;
  }
  vx= PA_KP_PILLAR_LOW * m_arc_position_error[0];
  vy= PA_KP_PILLAR_LOW * m_arc_position_error[1];
}

void RMChallengeFSM::navigateByTriangle(float &x, float &y, float &z)
{
  int triangle_sum= m_pillar_triangle[0] + m_pillar_triangle[1] +
                    m_pillar_triangle[2] + m_pillar_triangle[3];
  x= y= z= 0.0;
  float triangle_velocity= 0.0;

  /*use different velocity at different height*/
  if(m_prepare_to_land_type == PREPARE_AT_HIGH)
  {
    triangle_velocity= PA_LAND_TRIANGLE_VELOCITY_HIGH;
  }
  else if(m_prepare_to_land_type == PREPARE_AT_LOW)
  {
    triangle_velocity= PA_LAND_TRIANGLE_VELOCITY_LOW;
  }

  if(triangle_sum == 1)
  {
    if(m_pillar_triangle[0] == 1)
    {
      y= -triangle_velocity;
    }
    else if(m_pillar_triangle[1] == 1)
    {
      x= -triangle_velocity;
    }
    else if(m_pillar_triangle[2] == 1)
    {
      y= triangle_velocity;
    }
    else if(m_pillar_triangle[3] == 1)
    {
      x= triangle_velocity;
    }
  }
  else if(triangle_sum == 2)
  {
    if(m_pillar_triangle[0] == 1)
    {
      y= -triangle_velocity;
    }
    else if(m_pillar_triangle[2] == 1)
    {
      y= triangle_velocity;
    }
    if(m_pillar_triangle[1] == 1)
    {
      x= -triangle_velocity;
    }
    else if(m_pillar_triangle[3] == 1)
    {
      x= triangle_velocity;
    }
  }
  else if(triangle_sum == 3)
  {
    if(m_pillar_triangle[0] == 0)
    {
      y= triangle_velocity;
    }
    else if(m_pillar_triangle[1] == 0)
    {
      x= triangle_velocity;
    }
    else if(m_pillar_triangle[2] == 0)
    {
      y= -triangle_velocity;
    }
    else if(m_pillar_triangle[3] == 0)
    {
      x= -triangle_velocity;
    }
  }
}

void RMChallengeFSM::unitifyVector(float &x, float &y)
{
  float unit_x= x;
  float unit_y= y;
  x= unit_x / sqrt(pow(unit_x, 2) + pow(unit_y, 2));
  y= unit_y / sqrt(pow(unit_x, 2) + pow(unit_y, 2));
  // ROS_INFO_STREAM("vector is :" << x << "," << y);
}

/**
*This function transform global guidance position to
*uav local position, with a angle phi
*/
void RMChallengeFSM::transformCoordinate(float phi, float &x,
                                         float &y)
{
  float xw= x;
  float yw= y;
  x= xw * cos(phi) + yw * sin(phi);
  y= yw * cos(phi) - xw * sin(phi);
  // ROS_INFO_STREAM("transform coordinate is:" << x << "," << y);
}

void RMChallengeFSM::calculateNormalVelocity(float &x, float &y,
                                             LINE_TYPE line_type)
{
  if(line_type == VIRTUAL_LINE)
  {
    float xc= m_current_position_from_guidance[0];
    float yc= m_current_position_from_guidance[1];
    float x0= m_takeoff_points[m_current_takeoff_point_id][0];
    float y0= m_takeoff_points[m_current_takeoff_point_id][1];
    float xs= m_setpoints[m_current_takeoff_point_id][0];
    float ys= m_setpoints[m_current_takeoff_point_id][1];
    float t= ((xc - x0) * (xs - x0) + (yc - y0) * (ys - y0)) /
             (pow(xs - x0, 2) + pow(ys - y0, 2));
    float x_n= (x0 - xc) + t * (xs - x0);
    float y_n= (y0 - yc) + t * (ys - y0);
    unitifyVector(x_n, y_n);
    x= PA_KN * x_n;
    y= PA_KN * y_n;
  }
  else if(line_type == YELLOW_LINE)
  {
    unitifyVector(m_distance_to_line[0], m_distance_to_line[1]);
    x= PA_KN * m_distance_to_line[0];
    y= PA_KN * m_distance_to_line[1];
  }
}
void RMChallengeFSM::calculateTangentialVelocity(float &x, float &y,
                                                 LINE_TYPE line_type)
{
  if(line_type == VIRTUAL_LINE)
  {
    float x0= m_takeoff_points[m_current_takeoff_point_id][0];
    float y0= m_takeoff_points[m_current_takeoff_point_id][1];
    float xs= m_setpoints[m_current_takeoff_point_id][0];
    float ys= m_setpoints[m_current_takeoff_point_id][1];
    x= PA_KT * (xs - x0) / sqrt(pow(xs - x0, 2) + pow(ys - y0, 2));
    y= PA_KT * (ys - y0) / sqrt(pow(xs - x0, 2) + pow(ys - y0, 2));
  }
  else if(line_type == YELLOW_LINE)
  {
    unitifyVector(m_line_normal[0], m_line_normal[1]);
    if(m_current_takeoff_point_id == 3 ||
       m_current_takeoff_point_id == 4)
    {
      x= -PA_KT_RATIO*PA_KT * m_line_normal[0];
      y= -PA_KT_RATIO*PA_KT * m_line_normal[1];
    }
    else  // 0,1,2,5
    {
      x= PA_KT_RATIO*PA_KT * m_line_normal[0];
      y= PA_KT_RATIO*PA_KT * m_line_normal[1];
    }
  }
}
void RMChallengeFSM::calculateYawRate(float &yaw)
{
  float angle_to_line_1= 57.3 * acos(m_line_normal[0]);
  // float angle_to_line_2= 57.3 * acos(m_line_normal[0] * 0.5 -
  //                                    m_line_normal[1] *
  //                                    sqrt(0.75));

  ROS_INFO_STREAM("angle is:" << angle_to_line_1);

  if(fabs(angle_to_line_1) < PA_ANGLE_WITH_DIRECT_LINE_THRESHOLD)
  {
    if(fabs(angle_to_line_1) < PA_ANGLE_THRESHOLD)
    {
      yaw= m_line_normal[1] > 0 ? PA_YAW_RATE : -PA_YAW_RATE;
    }
    else
    {
      yaw= 0;
    }
  }
  else
  {
    yaw=0;
  }

  // ROS_INFO_STREAM("angle 1 and 2 are :" << angle_to_line_1 << ","
  //                                       << angle_to_line_2);
  // if(fabs(angle_to_line_1) < fabs(angle_to_line_2))
  // {
  //   if(fabs(angle_to_line_1) > PA_ANGLE_THRESHOLD)
  //   {
  //     yaw= m_line_normal[1] > 0 ? -PA_YAW_RATE : PA_YAW_RATE;
  //   }
  // }
  // else
  // {
  //   if(fabs(angle_to_line_2) > PA_ANGLE_THRESHOLD)
  //   {
  //     float sign=
  //         m_line_normal[0] * sqrt(0.75) + m_line_normal[1] * 0.5;
  //     yaw= sign > 0 ? -PA_YAW_RATE : PA_YAW_RATE;
  //   }
  // }
}
void RMChallengeFSM::setDroneState(int state)
{
  if(state == 1)
  {
    m_uav_state= UAV_LAND;
  }
  else if(state == 3)
  {
    m_uav_state= UAV_FLY;
  }
  ROS_INFO_STREAM("uav_state is:" << m_uav_state);
}
void RMChallengeFSM::setHeightFromGuidance(float height)
{
  m_current_height_from_guidance= height;
  ROS_INFO_STREAM(
      "height from guidance is:" << m_current_height_from_guidance);
}
/**
*set position from guidance
*when on land, bias of guidance position is updated
*when flying, use real time gudance position and bias to
*update actual guidance position
*/
void RMChallengeFSM::setPositionFromGuidance(float x, float y)
{
  transformCoordinate(PA_COORDINATE_TRANSFORM_ANGLE, x, y);
  if(m_uav_state == UAV_LAND)
  {
    /* update guidance bias*/
    m_guidance_bias[0]= x - m_current_position_from_guidance[0];
    m_guidance_bias[1]= y - m_current_position_from_guidance[1];
    ROS_INFO_STREAM("guidance bias is:" << m_guidance_bias[0] << ","
                                        << m_guidance_bias[1]);
  }
  else if(m_uav_state == UAV_FLY)
  {
    /* update actual position */
    m_current_position_from_guidance[0]= x - m_guidance_bias[0];
    m_current_position_from_guidance[1]= y - m_guidance_bias[1];
    ROS_INFO_STREAM("position from guidance is:"
                    << m_current_position_from_guidance[0] << ","
                    << m_current_position_from_guidance[1]);

    /*publish position*/
    geometry_msgs::Vector3Stamped pos;
    pos.header.frame_id= "position";
    pos.header.stamp= ros::Time::now();
    pos.vector.x= m_current_position_from_guidance[0];
    pos.vector.y= m_current_position_from_guidance[1];
    pos.vector.z= 0.0;
    m_position_pub.publish(pos);
  }
}

void RMChallengeFSM::setCircleVariables(bool is_circle_found,
                                        float position_error[2],
                                        float height)
{
  m_discover_pillar_circle= is_circle_found;
  if(is_circle_found)
  {
    m_circle_position_error[0]=
        position_error[0] - PA_CAMERA_DISPLACE;
    m_circle_position_error[1]= position_error[1];
    m_current_height_from_circle= height;
  }
  else
  {
    m_circle_position_error[0]= m_circle_position_error[1]=
        m_current_height_from_circle= 0;
  }
  // ROS_INFO_STREAM("circle var is:"
  //                 << m_discover_pillar_circle << ","
  //                 << m_circle_position_error[0] << ","
  //                 << m_circle_position_error[1] << ","
  //                 << m_current_height_from_circle);
}

void RMChallengeFSM::setTriangleVariables(int pillar_triangle[4])
{
  for(int i= 0; i < 4; ++i)
  {
    m_pillar_triangle[i]= pillar_triangle[i];
  }
  // ROS_INFO_STREAM("triangle is:" << pillar_triangle[0] << ","
  //                                << pillar_triangle[1] << ","
  //                                << pillar_triangle[2] << ","
  //                                << pillar_triangle[3]);
}

void RMChallengeFSM::setArcVariables(float position_error[2],
                                     float height)
{
  /*transform pixel position error to metric error*/
  calculateRealPositionError(position_error);

  m_arc_position_error[0]= position_error[0] - PA_CAMERA_DISPLACE;
  m_arc_position_error[1]= position_error[1];
  m_current_height_from_arc= height;
}

void RMChallengeFSM::calculateRealPositionError(float error[2])
{
  float z= m_current_height_from_guidance;
  float x= error[0], y= error[1];

  /*pin hole camera model*/
  error[0]= z * x / PA_CAMERA_F;
  error[1]= z * y / PA_CAMERA_F;
}

void RMChallengeFSM::setBaseVariables(bool is_base_found,
                                      float position_error[2])
{
  m_discover_base= is_base_found;
  if(is_base_found)
  {
    m_circle_position_error[0]= position_error[0];
    m_circle_position_error[1]= position_error[1];
  }
  else
  {
    m_circle_position_error[0]= 0;
    m_circle_position_error[1]= 0;
  }
  ROS_INFO_STREAM("base var is:" << m_discover_base << ","
                                 << m_circle_position_error[0] << ","
                                 << m_circle_position_error[1]);
}
void RMChallengeFSM::setLineVariables(float distance_to_line[2],
                                      float line_normal[2])
{
  m_distance_to_line[0]= distance_to_line[0];
  m_distance_to_line[1]= distance_to_line[1];
  m_line_normal[0]= line_normal[0];
  m_line_normal[1]= line_normal[1];
  ROS_INFO_STREAM("distance to line is:"
                  << m_distance_to_line[0] << ","
                  << m_distance_to_line[1] << ",line vector is"
                  << m_line_normal[0] << "," << m_line_normal[1]);
}
