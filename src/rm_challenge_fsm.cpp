#include "rm_challenge_fsm.h"

RMChallengeFSM::~RMChallengeFSM()
{
  delete m_serial_port;
#if CURRENT_COMPUTER == MANIFOLD
  m_drone->release_sdk_permission_control();
  delete m_drone;
#endif
}

void RMChallengeFSM::initialize(ros::NodeHandle &node_handle)
{
  /*initialize serial port*/
  m_serial_port= new boost::asio::serial_port(m_io_service);
  m_serial_port->open("/dev/ttyTHS0", m_err_code);
  m_serial_port->set_option(serial_port::baud_rate(9600), m_err_code);
  m_serial_port->set_option(
      serial_port::flow_control(serial_port::flow_control::none), m_err_code);
  m_serial_port->set_option(serial_port::parity(serial_port::parity::none),
                            m_err_code);
  m_serial_port->set_option(serial_port::stop_bits(serial_port::stop_bits::one),
                            m_err_code);
  m_serial_port->set_option(serial_port::character_size(8), m_err_code);

/*initialize dji sdk*/
#if CURRENT_COMPUTER == MANIFOLD
  m_drone= new DJIDrone(node_handle);
#endif

  /*initialize publisher*/
  m_position_pub=
      node_handle.advertise<geometry_msgs::Vector3Stamped>("/m100/position", 1);
  m_velocity_pub=
      node_handle.advertise<geometry_msgs::Vector3Stamped>("/m100/velocity", 1);
  m_color_change_pub=
      node_handle.advertise<std_msgs::String>("/tpp/color_change", 1);
  m_pillar_change_pub=
      node_handle.advertise<std_msgs::String>("/tpp/pillar_change", 1);
  m_line_change_pub=
      node_handle.advertise<std_msgs::String>("/tpp/line_change", 1);
  m_base_change_pub=
      node_handle.advertise<std_msgs::String>("/tpp/base_change", 1);

  /*initialize setpoint, takeoffpoint and takeoff height,
   only set for one time, takeoff positions are absolute position,
   while setpoint are relative position*/

  /*takeoff point absolute positions*/
  m_takeoff_points[PA_START][0]= 0.0;
  m_takeoff_points[PA_START][1]= 0.0;

  m_takeoff_points[PA_PILLAR_1][0]= 8.5;
  m_takeoff_points[PA_PILLAR_1][1]= 0.0;

  m_takeoff_points[PA_BASE_1][0]= 2.0;
  m_takeoff_points[PA_BASE_1][1]= 5.2;

  m_takeoff_points[PA_PILLAR_2][0]= 4.2;
  m_takeoff_points[PA_PILLAR_2][1]= 3.0;

  m_takeoff_points[PA_BASE_2][0]= 2.0;
  m_takeoff_points[PA_BASE_2][1]= 5.2;

  m_takeoff_points[PA_PILLAR_3][0]= 7.2;  //
  m_takeoff_points[PA_PILLAR_3][1]= 4.0;

  m_takeoff_points[PA_BASE_3][0]= 2.0;  //
  m_takeoff_points[PA_BASE_3][1]= 5.2;

  m_takeoff_points[PA_PILLAR_4][0]= 0.0;  //
  m_takeoff_points[PA_PILLAR_4][1]= 0.0;

  m_takeoff_points[PA_BASE_4][0]= 2.0;  //
  m_takeoff_points[PA_BASE_4][1]= 5.2;
  // qulification
  m_takeoff_points[PA_START_Q][0]= 0.0;  //
  m_takeoff_points[PA_START_Q][1]= 0.0;

  m_takeoff_points[PA_BASE_Q][0]= 2.0;  //
  m_takeoff_points[PA_BASE_Q][1]= 5.2;

  m_takeoff_points[PA_PILLAR_Q][0]= 8.5;  //
  m_takeoff_points[PA_PILLAR_Q][1]= 3.0;

  /*set point RELATIVE position*/
  m_setpoints[PA_START][0]= 8.5;
  m_setpoints[PA_START][1]= 0.0;

  m_setpoints[PA_PILLAR_1][0]= -2.0;
  m_setpoints[PA_PILLAR_1][1]= -1.2;

  m_setpoints[PA_BASE_1][0]= 4.2;
  m_setpoints[PA_BASE_1][1]= 0.0;

  m_setpoints[PA_PILLAR_2][0]= -1.8;
  m_setpoints[PA_PILLAR_2][1]= 1.8;

  m_setpoints[PA_BASE_2][0]= 4.2;  //
  m_setpoints[PA_BASE_2][1]= 0.0;

  m_setpoints[PA_PILLAR_3][0]= -1.8;  //
  m_setpoints[PA_PILLAR_3][1]= 1.8;

  m_setpoints[PA_BASE_3][0]= 0.0;  //
  m_setpoints[PA_BASE_3][1]= 0.0;

  m_setpoints[PA_PILLAR_4][0]= 0.0;  //
  m_setpoints[PA_PILLAR_4][1]= 0.0;

  m_setpoints[PA_BASE_4][0]= 0.0;  //
  m_setpoints[PA_BASE_4][1]= 0.0;
  // qulification
  m_setpoints[PA_START_Q][0]= 4.0;  //
  m_setpoints[PA_START_Q][1]= 0.0;

  m_setpoints[PA_BASE_Q][0]= 3.0;  //
  m_setpoints[PA_BASE_Q][1]= 0.0;

  m_setpoints[PA_PILLAR_Q][0]= 0.0;  //
  m_setpoints[PA_PILLAR_Q][1]= 0.0;

  /*take off height*/
  // This should minus start point height later!
  m_goal_height[PA_START]= PA_TAKEOFF_HEIGHT - PA_TARMAC_HEIGHT;
  m_goal_height[PA_PILLAR_1]=
      PA_TAKEOFF_HEIGHT - PA_PILLAR_HEIGHT - PA_BRIDGE_HEIGHT;
  m_goal_height[PA_PILLAR_2]= PA_TAKEOFF_HEIGHT - PA_PILLAR_HEIGHT;
  m_goal_height[PA_PILLAR_3]= PA_TAKEOFF_HEIGHT - PA_PILLAR_HEIGHT;
  m_goal_height[PA_PILLAR_4]= PA_TAKEOFF_HEIGHT - PA_PILLAR_HEIGHT;
  m_goal_height[PA_BASE_1]= PA_TAKEOFF_HEIGHT;
  m_goal_height[PA_BASE_2]= PA_TAKEOFF_HEIGHT;
  m_goal_height[PA_BASE_3]= PA_TAKEOFF_HEIGHT - PA_BRIDGE_HEIGHT;
  m_goal_height[PA_BASE_4]= PA_TAKEOFF_HEIGHT;
  // qulification
  m_goal_height[PA_START_Q]=
      PA_TAKEOFF_HEIGHT - PA_TARMAC_HEIGHT - PA_BRIDGE_HEIGHT;
  m_goal_height[PA_BASE_Q]= PA_TAKEOFF_HEIGHT;
  m_goal_height[PA_PILLAR_Q]= PA_TAKEOFF_HEIGHT - PA_PILLAR_HEIGHT;

  /*initialize  state*/
  resetAllState();
}

void RMChallengeFSM::resetAllState()
{
  ros::Duration(1.0).sleep();
  m_state= TAKE_OFF;
  m_uav_state= UAV_LAND;
  m_prepare_to_land_type= PREPARE_AT_HIGH;
  m_graspper_control_time= 0;
  /*if want to test different task,change id here as well as .h*/
  m_current_takeoff_point_id= PA_BASE_2;
  /**/
  droneUpdatePosition();

  m_land_counter= 0;
#if CURRENT_COMPUTER == MANIFOLD
  m_drone->request_sdk_permission_control();
#endif
}

void RMChallengeFSM::run()
{
  printStateInfo();
  switch(m_state)
  {
    case TAKE_OFF:
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
          updatePillarColor();
          transferToTask(GO_UP);
        }
      }
      break;
    }

    case GO_UP:
    {
      if(!closeToGoalHeight())
      {
        droneGoUp();
      }
      else if(closeToGoalHeight())
      {
        transferToTask(GO_TO_SETPOINT);
      }
      break;
    }

    /*do no vision task when go to set point, only judge if close to
    set point, when close, either go to track line or idle*/
    case GO_TO_SETPOINT:
    {
      updateVisionTask();
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
      break;
    }

    case TRACK_LINE:
    {
      if(!discoverLandPoint())
      {
        if(discoverYellowLine())
        {
          droneTrackLine();
          if(discoverT())
          {
            if(nextTargetIsClosePillar())
            {
              // droneUpdatePosition(PA_T_1);
              transferToTask(GO_TO_PILLAR);
            }
            else if(nextTargetIsFarPillar())
            {
              /*go forward for a while*/
              updateTPosition();
              transferToTask(TRACK_LINE_FORWARD);
            }
            else if(nextTargetIsBase())
            {
              updateTPosition();
              transferToTask(TRACK_LINE_BACKWARD);
            }
          }
          /*
          //This part should be added later,T
          if(discoverT())
          {
            if(nextTargetIsClosePillar())
            {
              transferToTask(GO_TO_PILLAR);
            }
            else if(nextTargetIsFarPillar())
            {
              transferToTask(CROSS_ARENA)
            }
          }
          */
        }
        else
        {
          if(!closeToSetPoint())
          {
            transferToTask(GO_TO_SETPOINT);
          }
          else
          {
            transferToTask(IDLE);
          }
        }
      }
      else
      {
        transferToTask(GO_TO_LAND_POINT);
      }
      break;
    }

    case GO_TO_LAND_POINT:
    {
      if(stillFindLandPoint())
      {
        if(!readyToLand())
        {
          dronePrepareToLand();
        }
        else
        {
          if(landPointIsPillar())
          {
            droneDropDown();
            transferToTask(LAND);
          }
          else if(landPointIsBase())
          {
            if(!isTheLastTravel())
            {
              transferToTask(RELEASE_BALL);
            }
            else if(isQulifying())
            {
              // transferToTask(LAND);
              updateTakeoffPointId();
              droneUpdatePosition();
              transferToTask(GO_UP);
            }
            else
            {
              transferToTask(LAND);
            }
          }
        }
      }
      else
      {
        if(!closeToSetPoint())
        {
          transferToTask(GO_TO_SETPOINT);
        }
        else
        {
          transferToTask(IDLE);
        }
      }
      break;
    }

    case LAND:
    {
      if(!isOnLand())
      {
        /* continue to land */
        openGraspper();
        droneLand();
      }
      else if(isOnLand())
      {
        transferToTask(GRAB_BALL);
      }
      break;
    }

    case GRAB_BALL:
    {
      if(!finishGrabBallTask())
      {
        /* continue graspper control */
        grabBall();
      }
      else
      {
        closeGraspper();
        updateTakeoffPointId();
        droneUpdatePosition();
        transferToTask(TAKE_OFF);
      }
      break;
    }

    case IDLE:
    {
      if(discoverLandPoint())
      {
        transferToTask(GO_TO_LAND_POINT);
      }
      else if(discoverYellowLine())
      {
        transferToTask(TRACK_LINE);
      }
      else
      {
        // do nothing, wait
        droneHover();
        ros::Duration(0.5).sleep();
      }
      break;
    }

    case RELEASE_BALL:
    {
      /*go down to lower height*/
      if(lowEnoughToReleaseBall())
      {
        droneHover();
        droneReleaseBall();
        updateTakeoffPointId();
        droneUpdatePosition();
        transferToTask(GO_UP);
      }
      else
      {
        droneGoDownToBase();
      }
      break;
    }

    case CROSS_ARENA:
    {
      break;
    }

    case GO_TO_PILLAR:
    {
      droneGoToPillar();
      if(discoverLandPoint())
      {
        transferToTask(GO_TO_LAND_POINT);
      }
      break;
    }

    case TRACK_LINE_FORWARD:
    {
      if(!forwardFarEnough())
      {
        // droneTrackLine();
        controlDroneVelocity(PA_KT, 0.0, 0.0, 0.0);
        ROS_INFO("forward!");
      }
      else
      {
        transferToTask(TRACK_LINE);
      }
      break;
    }

    case TRACK_LINE_BACKWARD:
    {
      if(!backwardFarEnough())
      {
        controlDroneVelocity(-PA_KT, 0.0, 0.0, 0.0);
        ROS_INFO("backward!");
      }
      else
      {
        transferToTask(TRACK_LINE);
      }
      break;
    }
  }
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
  else if(task_state == GRAB_BALL)
  {
    m_state= GRAB_BALL;
  }
  else if(task_state == GO_TO_LAND_POINT)
  {
    m_state= GO_TO_LAND_POINT;
  }
  else if(task_state == GO_TO_PILLAR)
  {
    m_state= GO_TO_PILLAR;
  }
  else if(task_state == RELEASE_BALL)
  {
    m_state= RELEASE_BALL;
  }
  else if(task_state == CROSS_ARENA)
  {
    m_state= CROSS_ARENA;
  }
  else if(task_state == TRACK_LINE_FORWARD)
  {
    m_state= TRACK_LINE_FORWARD;
  }
  else if(task_state == TRACK_LINE_BACKWARD)
  {
    m_state= TRACK_LINE_BACKWARD;
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
    ROS_INFO_STREAM("Take off height error :" << height_error << "is "
                                                                 "small");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("Take off height error:" << height_error << "is too "
                                                                "large");
    return false;
  }
}

bool RMChallengeFSM::farFromTakeoffPoint()
{
  double pos_error= sqrt(
      pow(m_real_position[0] - m_takeoff_points[m_current_takeoff_point_id][0],
          2) +
      pow(m_real_position[1] - m_takeoff_points[m_current_takeoff_point_id][1],
          2));
  ROS_INFO_STREAM("guidance position:" << m_real_position[0] << " "
                                       << m_real_position[1]);
  ROS_INFO_STREAM("takeoff position:"
                  << m_takeoff_points[m_current_takeoff_point_id][0] << " "
                  << m_takeoff_points[m_current_takeoff_point_id][1]);

  if(pos_error > PA_TAKEOFF_POSITION_ERROR)
  {
    ROS_INFO_STREAM("distance to takeoff point is:" << pos_error << ",fa"
                                                                    "r");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("distance to takeoff point is:" << pos_error << ",too "
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
  if(m_current_takeoff_point_id == PA_PILLAR_1 ||
     m_current_takeoff_point_id == PA_PILLAR_2 ||
     m_current_takeoff_point_id == PA_PILLAR_3 ||
     m_current_takeoff_point_id == PA_PILLAR_4 ||
     m_current_takeoff_point_id == PA_START_Q)
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
    bool is_pillar_found= m_discover_pillar_circle || discoverTriangle();
    float landpoint_error=
        sqrt(pow(m_real_position[0] -
                     m_takeoff_points[m_current_takeoff_point_id + 1][0],
                 2) +
             pow(m_real_position[1] -
                     m_takeoff_points[m_current_takeoff_point_id + 1][1],
                 2));
    bool close_to_lp=
        landpoint_error < PA_LANDPOINT_POSITION_ERROR ? true : false;

    if(!is_pillar_found)
    {
      ROS_INFO_STREAM("no circle from vision");
      return false;
    }
    else if(!close_to_lp)
    {
      ROS_INFO_STREAM("far from landpoint");
      ROS_INFO_STREAM("guidance position:" << m_real_position[0] << " "
                                           << m_real_position[1]);
      ROS_INFO_STREAM("takeoff position:"
                      << m_takeoff_points[m_current_takeoff_point_id + 1][0]
                      << " "
                      << m_takeoff_points[m_current_takeoff_point_id + 1][1]);
      return false;
    }
    else
    {
      m_land_point_type= PILLAR_LAND_POINT;
      ROS_INFO_STREAM("circle:" << m_discover_pillar_circle);
      return true;
    }
  }
}

bool RMChallengeFSM::stillFindLandPoint()
{
  if(m_current_takeoff_point_id == PA_PILLAR_1 ||
     m_current_takeoff_point_id == PA_PILLAR_2 ||
     m_current_takeoff_point_id == PA_PILLAR_3 ||
     m_current_takeoff_point_id == PA_PILLAR_4 ||
     m_current_takeoff_point_id == PA_START_Q)
  {
    if(m_discover_base)
    {
      m_land_point_type= BASE_LAND_POINT;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(m_discover_pillar_circle || discoverTriangle() ||
       m_discover_pillar_arc || m_prepare_to_land_type != PREPARE_AT_HIGH)
    {
      m_land_point_type= PILLAR_LAND_POINT;
      return true;
    }
    else
    {
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
  float disp_x=
      m_real_position[0] - m_takeoff_points[m_current_takeoff_point_id][0];
  float disp_y=
      m_real_position[1] - m_takeoff_points[m_current_takeoff_point_id][1];
  double pos_error=
      sqrt(pow(disp_x - m_setpoints[m_current_takeoff_point_id][0], 2) +
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
bool RMChallengeFSM::finishGrabBallTask()
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

void RMChallengeFSM::grabBall()
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

void RMChallengeFSM::controlDroneVelocity(float x, float y, float z, float yaw)
{
#if CURRENT_COMPUTER == MANIFOLD
  m_drone->attitude_control(0x4B, x, y, z, yaw);
#endif
  ros::Duration(20 / 1000).sleep();
}

void RMChallengeFSM::droneGoUp()
{
  if(m_goal_height[m_current_takeoff_point_id] > m_current_height_from_guidance)
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
  float vt_x= 0, vt_y= 0;
  calculateTangentialVelocity(vt_x, vt_y, VIRTUAL_LINE_SETPOINT);
  ROS_INFO_STREAM("vtx:" << vt_x << " vt_y:" << vt_y);
  float vn_x= 0, vn_y= 0;
  calculateNormalVelocity(vn_x, vn_y, VIRTUAL_LINE_SETPOINT);
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
  float vt_x= 0, vt_y= 0;
  float vn_x= 0, vn_y= 0;
  float v_z= 0, yaw= 0;

  calculateTangentialVelocity(vt_x, vt_y, YELLOW_LINE);
  calculateNormalVelocity(vn_x, vn_y, YELLOW_LINE);
  calculateZVelocity(v_z);
  calculateYawRate(yaw);
  ROS_INFO_STREAM("\n t:" << vt_x << "," << vt_y << "\n"
                          << "n:" << vn_x << "," << vn_y << "\n"
                          << "yaw:" << yaw << "\n"
                          << "vz:" << v_z);
  // controlDroneVelocity(vt_x + vn_x, vt_y + vn_y, v_z, yaw);
  controlDroneVelocity(vt_x + vn_x, vt_y + vn_y, v_z, 0.0);
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
    controlDroneVelocity(0.0, 0.0, -0.9, 0.0);
}

bool RMChallengeFSM::readyToLand()
{
  float land_err= sqrt(pow(m_circle_position_error[0], 2) +
                       pow(m_circle_position_error[1], 2));
  float height_error;
  if(m_land_point_type == BASE_LAND_POINT)
  {
    /*only calculate position error*/
    float pos_error= sqrt(pow(m_base_position_error[0], 2) +
                          pow(m_base_position_error[1], 2));
    if(pos_error < PA_BASE_POSITION_THRESHOLD && m_discover_base)
    {
      // ROS_INFO_STREAM("ready to land at base," << land_err << ","
      //                                          << height_error);
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
    height_error= fabs(PA_LAND_HEIGHT_FINAL - m_current_height_from_guidance);
    float oror=
        sqrt(pow(m_arc_position_error[0], 2) + pow(m_arc_position_error[1], 2));
    float pos_error_x= fabs(m_arc_position_error[0]);
    float pos_error_y= fabs(m_arc_position_error[1]);
    // need output
    if(m_prepare_to_land_type == PREPARE_AT_SUPER_LOW &&
       pos_error_x < PA_LAND_POSITION_THRESHOLD_SUPER_LOW &&
       pos_error_y < PA_LAND_POSITION_THRESHOLD_SUPER_LOW &&
       height_error < PA_LAND_HEIGHT_THRESHOLD_FINAL)
    {
      droneHover();
      if(isCheckedTimeSuitable())
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      updateCheckedTime();
      return false;
    }
  }
}

bool RMChallengeFSM::isCheckedTimeSuitable()
{
  ros::Time now= ros::Time::now();
  double d= now.toSec() - m_checked_time.toSec();
  if(d < PA_TIME_MAX && d > PA_TIME_MIN)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void RMChallengeFSM::updateCheckedTime()
{
  m_checked_time= ros::Time::now();
}

void RMChallengeFSM::dronePrepareToLand()
{
  float vx= 0, vy= 0, vz= 0;
  std::string velocity_id;
  if(m_land_point_type == BASE_LAND_POINT)
  {
    /*adjust position to center of base
      height and position
    */
    if(fabs(m_current_height_from_guidance - PA_BASE_HEIGHT) >
       PA_BASE_HEIGHT_THRESHOLD)
    {
      vz= PA_BASE_HEIGHT > m_current_height_from_guidance ?
              PA_FLYING_Z_VELOCITY :
              -PA_FLYING_Z_VELOCITY;
      // vx= vy= 0;
    }
    else
    {
      vz= 0;
    }
    vx= -PA_KP_BASE * m_base_position_error[0];
    vy= -PA_KP_BASE * m_base_position_error[1];
    ROS_INFO_STREAM("landing at base v are:" << vx << "," << vy << "," << vz);
  }
  else if(m_land_point_type == PILLAR_LAND_POINT)
  {
    if(m_prepare_to_land_type == PREPARE_AT_SUPER_LOW)
    {
      navigateByArc(vx, vy, vz);
      velocity_id= "by arc";
      publishLineChange("pause");
    }
    else if(m_discover_pillar_circle)
    {
      navigateByCircle(vx, vy, vz);
      velocity_id= "by circle";
      publishLineChange("pause");
    }
    else if(discoverTriangle())
    {
      navigateByTriangle(vx, vy, vz);
      velocity_id= "by triangle";
      // ROS_INFO_STREAM("navigate by triangle");
    }
    else
    {
      // ROS_INFO_STREAM("Miss pillar!!!");
      velocity_id= "miss pillar";
    }
    // ROS_INFO_STREAM("landing v at pillar are:" << vx << "," << vy
    //                                            << "," << vz);
  }
  controlDroneVelocity(vx, vy, vz, 0.0);

  /*publish velocity*/
  publishVelocity(velocity_id, vx, vy, vz);
  if(velocity_id == "by arc")
  {
    /*publish position and height error to compare*/
    publishVelocity("arc error", m_arc_position_error[0],
                    m_arc_position_error[1], m_current_height_from_guidance);
  }
}

void RMChallengeFSM::publishVelocity(std::string id, float x, float y, float z)
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
      float height_error= PA_LAND_HEIGHT - m_current_height_from_circle;
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
  float height_error= PA_LAND_HEIGHT_FINAL - m_current_height_from_guidance;
  float pos_error=
      sqrt(pow(m_arc_position_error[0], 2) + pow(m_arc_position_error[1], 2));
  float pos_error_x= m_arc_position_error[0];
  float pos_error_y= m_arc_position_error[1];
  if(fabs(pos_error_x) > PA_LAND_POSITION_THRESHOLD_SUPER_LOW_BIG ||
     fabs(pos_error_y) > PA_LAND_POSITION_THRESHOLD_SUPER_LOW_BIG)
  {
    vz= 0;
    vx= fabs(pos_error_x) > PA_LAND_POSITION_THRESHOLD_SUPER_LOW_BIG ?
            PA_KP_PILLAR_LOW * m_arc_position_error[0] :
            0;
    vy= fabs(pos_error_y) > PA_LAND_POSITION_THRESHOLD_SUPER_LOW_BIG ?
            PA_KP_PILLAR_LOW * m_arc_position_error[1] :
            0;
  }
  else if(fabs(height_error) > PA_LAND_HEIGHT_THRESHOLD_FINAL)
  {
    /*height error too big, use height from guidance to navigate*/
    vz= fabs(height_error) / (height_error + 0.0000000001) *
        PA_LAND_Z_VELOCITY_FINAL;
    vx= fabs(pos_error_x) > PA_LAND_POSITION_THRESHOLD_SUPER_LOW ?
            PA_KP_PILLAR_LOW * m_arc_position_error[0] :
            0;
    vy= fabs(pos_error_y) > PA_LAND_POSITION_THRESHOLD_SUPER_LOW ?
            PA_KP_PILLAR_LOW * m_arc_position_error[1] :
            0;
  }
  else
  {
    vz= 0;
    vx= PA_V_MIN_FINAL * fabs(m_arc_position_error[0]) /
        (m_arc_position_error[0] + 0.00000001);
    vy= PA_V_MIN_FINAL * fabs(m_arc_position_error[1]) /
        (m_arc_position_error[1] + 0.00000001);
  }
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
void RMChallengeFSM::transformCoordinate(float phi, float &x, float &y)
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
  if(line_type == VIRTUAL_LINE_SETPOINT)
  {
    float xc= m_real_position[0];
    float yc= m_real_position[1];
    float x0= m_takeoff_points[m_current_takeoff_point_id][0];
    float y0= m_takeoff_points[m_current_takeoff_point_id][1];
    float xs= x0 + m_setpoints[m_current_takeoff_point_id][0];
    float ys= y0 + m_setpoints[m_current_takeoff_point_id][1];
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
  if(line_type == VIRTUAL_LINE_SETPOINT)
  {
    float x0= m_takeoff_points[m_current_takeoff_point_id][0];
    float y0= m_takeoff_points[m_current_takeoff_point_id][1];
    float xs= x0 + m_setpoints[m_current_takeoff_point_id][0];
    float ys= y0 + m_setpoints[m_current_takeoff_point_id][1];
    x= PA_KT * (xs - x0) / sqrt(pow(xs - x0, 2) + pow(ys - y0, 2));
    y= PA_KT * (ys - y0) / sqrt(pow(xs - x0, 2) + pow(ys - y0, 2));
  }

  else if(line_type == YELLOW_LINE)
  {
    unitifyVector(m_line_normal[0], m_line_normal[1]);
    judgeLineDirection();

    x= PA_KT_RATIO * PA_KT * m_line_normal[0];
    y= PA_KT_RATIO * PA_KT * m_line_normal[1];
  }

  else if(line_type == VIRTUAL_LINE_LANDPOINT)
  {
    float x0= m_real_position[0];
    float y0= m_real_position[1];
    float xs= m_takeoff_points[m_current_takeoff_point_id + 1][0];
    float ys= m_takeoff_points[m_current_takeoff_point_id + 1][1];
    x= PA_KT * (xs - x0) / sqrt(pow(xs - x0, 2) + pow(ys - y0, 2));
    y= PA_KT * (ys - y0) / sqrt(pow(xs - x0, 2) + pow(ys - y0, 2));
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
    yaw= 0;
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
  ROS_INFO_STREAM("height from guidance is:" << m_current_height_from_guidance);
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
  m_raw_guidance_position[0]= x;
  m_raw_guidance_position[1]= y;
  if(m_uav_state == UAV_LAND)
  {
    /* update guidance bias*/
    m_guidance_bias[0]= x - m_real_position[0];
    m_guidance_bias[1]= y - m_real_position[1];
    ROS_INFO_STREAM("guidance bias is:" << m_guidance_bias[0] << ","
                                        << m_guidance_bias[1]);
  }
  else if(m_uav_state == UAV_FLY)
  {
    /* update actual position */
    m_real_position[0]= x - m_guidance_bias[0];
    m_real_position[1]= y - m_guidance_bias[1];
    ROS_INFO_STREAM("position from guidance is:" << m_real_position[0] << ","
                                                 << m_real_position[1]);

    /*publish position*/
    geometry_msgs::Vector3Stamped pos;
    pos.header.frame_id= "position";
    pos.header.stamp= ros::Time::now();
    pos.vector.x= m_real_position[0];
    pos.vector.y= m_real_position[1];
    pos.vector.z= 0.0;
    m_position_pub.publish(pos);
  }
}

void RMChallengeFSM::setCircleVariables(bool is_circle_found,
                                        float position_error[2], float height)
{
  m_discover_pillar_circle= is_circle_found;
  if(is_circle_found)
  {
    m_circle_position_error[0]= position_error[0] - PA_CAMERA_DISPLACE;
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

void RMChallengeFSM::setArcVariables(bool is_arc_found, float position_error[2])
{
  /*transform pixel position error to metric error*/
  calculateRealPositionError(position_error);

  m_arc_position_error[0]= position_error[0] - PA_CAMERA_DISPLACE;
  m_arc_position_error[1]= position_error[1];
  m_discover_pillar_arc= is_arc_found;
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
                                      float position_error[2], float base_angle)
{
  m_discover_base= is_base_found;
  if(is_base_found)
  {
    m_base_position_error[0]= position_error[0] - PA_CAMERA_DISPLACE;
    m_base_position_error[1]= position_error[1];
    m_base_angle=base_angle;
  }
  else
  {
    m_base_position_error[0]= 0;
    m_base_position_error[1]= 0;
    m_base_angle=base_angle;
  }
  // ROS_INFO_STREAM("base var is:" << m_discover_base << ","
  //                                << m_base_position_error[0] << ","
  //                                << m_base_position_error[1]);
}
void RMChallengeFSM::setLineVariables(bool is_T_found,
                                      float distance_to_line[2],
                                      float line_normal[2])
{
  m_discover_T= is_T_found;
  m_distance_to_line[0]= distance_to_line[0];
  m_distance_to_line[1]= distance_to_line[1];
  m_line_normal[0]= line_normal[0];
  m_line_normal[1]= line_normal[1];
  // ROS_INFO_STREAM("distance to line is:"
  //                 << m_distance_to_line[0] << ","
  //                 << m_distance_to_line[1] << ",line vector is"
  //                 << m_line_normal[0] << "," << m_line_normal[1]);
}

bool RMChallengeFSM::landPointIsPillar()
{
  if(m_current_takeoff_point_id == PA_START ||
     m_current_takeoff_point_id == PA_BASE_1 ||
     m_current_takeoff_point_id == PA_BASE_2 ||
     m_current_takeoff_point_id == PA_BASE_3 ||
     m_current_takeoff_point_id == PA_BASE_Q)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool RMChallengeFSM::landPointIsBase()
{
  if(m_current_takeoff_point_id == PA_PILLAR_1 ||
     m_current_takeoff_point_id == PA_PILLAR_2 ||
     m_current_takeoff_point_id == PA_PILLAR_3 ||
     m_current_takeoff_point_id == PA_PILLAR_4 ||
     m_current_takeoff_point_id == PA_START_Q)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*id:
  0:uav start point
  1,3,5,7:pillar
  2,4,6:base
id==5:only fly to 3 pillar
*/

bool RMChallengeFSM::isTheLastTravel()
{
  if(m_current_takeoff_point_id == PA_PILLAR_3)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool RMChallengeFSM::nextTargetIsClosePillar()
{
  if(m_current_takeoff_point_id == PA_BASE_1)
    return true;
  else
    return false;
}

bool RMChallengeFSM::nextTargetIsFarPillar()
{
  if(m_current_takeoff_point_id == PA_BASE_2 ||
     m_current_takeoff_point_id == PA_BASE_Q)
    return true;
  else
    return false;
}

bool RMChallengeFSM::lowEnoughToReleaseBall()
{
  if(fabs(m_current_height_from_guidance - PA_RELEASE_BALL_HEIGHT) >
     PA_RELEASE_BALL_HEIGHT_THRESHOLD)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void RMChallengeFSM::droneReleaseBall()
{
  openGraspper();
  ros::Duration(1.5).sleep();
  closeGraspper();
}

void RMChallengeFSM::droneGoDownToBase()
{
  /*go down faster when height is large,
   slower when height is small*/
  float vz= PA_RELEASE_BALL_HEIGHT > m_current_height_from_guidance ?
                PA_RELEASE_BALL_VELOCITY :
                -PA_RELEASE_BALL_VELOCITY;
  if((m_current_height_from_guidance - PA_RELEASE_BALL_HEIGHT) >
     PA_SLOW_DOWN_HEIGHT)
  {
    vz*= 4;
  }
  controlDroneVelocity(0.0, 0.0, vz, 0.0);
}

void RMChallengeFSM::updateTakeoffPointId()
{
  m_current_takeoff_point_id++;
}

bool RMChallengeFSM::discoverT()
{
  /*
  judging citeria:
    discover T from vision,
    correct takeoff point id,
    distance to landpoint close enough
  */
  ROS_INFO_STREAM("looking for T");
  bool is_takeoff_id_correct= m_current_takeoff_point_id == PA_BASE_1 ||
                              m_current_takeoff_point_id == PA_BASE_2 ||
                              m_current_takeoff_point_id == PA_BASE_Q;
  if(!is_takeoff_id_correct)
  {
    ROS_INFO_STREAM("ID wrong");
    return false;
  }

  if(!m_discover_T)
  {
    ROS_INFO_STREAM("no T from vision");
    return false;
  }

  float pos_error=
      sqrt(pow(m_real_position[0] -
                   m_takeoff_points[PA_PILLAR_2][0],
               2) +
           pow(m_real_position[1] -
                   m_takeoff_points[PA_PILLAR_2][1] -
                   PA_T_DISPLACE,
               2));
  bool is_close_to_target=
      pos_error < PA_LANDPOINT_POSITION_ERROR ? true : false;

  if(is_close_to_target)
  {
    ROS_INFO_STREAM("TTTTTTTTTTTTTTTTTTTT");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("too far from T");
    return false;
  }
}

void RMChallengeFSM::droneGoToPillar()
{
  /*go to takeoff point +1*/
  float vt_x= 0, vt_y= 0;
  calculateTangentialVelocity(vt_x, vt_y, VIRTUAL_LINE_LANDPOINT);
  controlDroneVelocity(vt_x, vt_y, 0.0, 0.0);
}

void RMChallengeFSM::droneUpdatePosition(int POSITION_ID)
{
  if(POSITION_ID == 0)
  {
    POSITION_ID= m_current_takeoff_point_id;
  }
  /*set pisition from guidance to according point,then update bias */
  if(POSITION_ID <= PA_PILLAR_Q)
  {
    m_real_position[0]= m_takeoff_points[POSITION_ID][0];
    m_real_position[1]= m_takeoff_points[POSITION_ID][1];
    m_guidance_bias[0]= m_raw_guidance_position[0] - m_real_position[0];
    m_guidance_bias[1]= m_raw_guidance_position[1] - m_real_position[1];
  }
  /*else if(POSITION_ID == PA_T_1)
  {
    m_real_position[0]= m_takeoff_points[PA_PILLAR_2][0];
    m_real_position[1]=
        m_takeoff_points[PA_PILLAR_2][1] + PA_T_DISPLACE;
  }
  else if(POSITION_ID == PA_T_2)
  {
    m_real_position[0]= m_takeoff_points[PA_PILLAR_4][0];
    m_real_position[1]=
        m_takeoff_points[PA_PILLAR_4][1] - PA_T_DISPLACE;
  }*/
}

void RMChallengeFSM::printStateInfo()
{
  std::string s;
  switch(m_state)
  {
    case TAKE_OFF:
    {
      s= "TAKE OFF";
      break;
    }
    case GO_UP:
    {
      s= "GO UP";
      break;
    }
    case GO_TO_SETPOINT:
    {
      s= "GO TO SETPOINT";
      break;
    }
    case IDLE:
    {
      s= "IDLE";
      break;
    }
    case TRACK_LINE:
    {
      s= "TRACK LINE";
      break;
    }
    case LAND:
    {
      s= "LAND";
      break;
    }
    case GRAB_BALL:
    {
      s= "GRAB BALL";
      break;
    }
    case GO_TO_LAND_POINT:
    {
      s= "GO TO LANDPOINT";
      break;
    }
    case GO_TO_PILLAR:
    {
      s= "GO TO PILLAR";
      break;
    }
    case RELEASE_BALL:
    {
      s= "RELEASE BALL";
      break;
    }
    case CROSS_ARENA:
    {
      s= "CROSS ARENA";
      break;
    }
    case TRACK_LINE_FORWARD:
    {
      s= "TRACK LINE FORWARD";
      break;
    }
    case TRACK_LINE_BACKWARD:
    {
      s= "TRACK LINE BACKWARD";
      break;
    }
  }
  ROS_INFO_STREAM("\n State is: " << s);
}

void RMChallengeFSM::judgeLineDirection()
{
  float x, y;
  float dot;
  switch(m_current_takeoff_point_id)
  {
    case PA_START:
    {
      x= sqrt(3) / 2;
      y= -1 / 2;
      break;
    }
    case PA_PILLAR_1:
    {
      x= -sqrt(3) / 2;
      y= 1 / 2;
      break;
    }
    case PA_BASE_1:
    {
      x= 1;
      y= 0;
      break;
    }
    case PA_PILLAR_2:
    {
      x= -1;
      y= 0;
      break;
    }
    case PA_BASE_2:
    {
      x= 1;
      y= 0;
      break;
    }
    case PA_PILLAR_3:
    {
      x= -1;
      y= 0;
      break;
    }
    case PA_BASE_3:
    {
      x= 1;
      y= 0;
      break;
    }
    case PA_PILLAR_4:
    {
      x= -1;
      y= 0;
      break;
    }
    case PA_BASE_4:
    {
      break;
    }
    /*qulifying contest*/
    case PA_START_Q:
    {
      x= -sqrt(3) / 2;
      y= 1 / 2;
      break;
    }
    case PA_BASE_Q:
    {
      x= 1;
      y= 0;
      break;
    }
    case PA_PILLAR_Q:
    {
      break;
    }
  }

  dot= m_line_normal[0] * x + m_line_normal[1] * y;
  if(dot < 0)
  {
    m_line_normal[0]= -m_line_normal[0];
    m_line_normal[1]= -m_line_normal[1];
  }
}

void RMChallengeFSM::publishColorChange()
{
  std_msgs::String msg;
  msg.data= "";
  m_color_change_pub.publish(msg);
  ROS_INFO_STREAM("inform camera pillar color change");
}

void RMChallengeFSM::publishPillarChange(std::string state)
{
  if(state == "resume" || state == "pause")
  {
    std_msgs::String msg;
    msg.data= state;
    m_pillar_change_pub.publish(msg);
    ROS_INFO_STREAM("info pillar task change");
  }
  else
  {
    ROS_INFO_STREAM("invalid state change");
  }
}

void RMChallengeFSM::publishLineChange(std::string state)
{
  if(state == "resume" || state == "pause")
  {
    std_msgs::String msg;
    msg.data= state;
    m_line_change_pub.publish(msg);
    ROS_INFO_STREAM("info line task change");
  }
  else
  {
    ROS_INFO_STREAM("invalid state change");
  }
}

void RMChallengeFSM::publishBaseChange(std::string state)
{
  if(state == "resume" || state == "pause")
  {
    std_msgs::String msg;
    msg.data= state;
    m_base_change_pub.publish(msg);
    ROS_INFO_STREAM("info base task change");
  }
  else
  {
    ROS_INFO_STREAM("invalid state change");
  }
}

void RMChallengeFSM::updatePillarColor()
{
  /*when at pillar 1 and pillar 3,need to change pillar color*/
  if(m_current_takeoff_point_id == PA_PILLAR_1 ||
     m_current_takeoff_point_id == PA_PILLAR_3)
  {
    publishColorChange();
  }
}

void RMChallengeFSM::calculateZVelocity(float &vz)
{
  /*
  pillar1:track line with lower height(1.5 to floor)
  base3:track line with lower height
    and go higher when see direct line
  */
  vz= 0;

  switch(m_current_takeoff_point_id)
  {
    case PA_PILLAR_1:
    {
      if(fabs(m_current_height_from_guidance - PA_FLYING_HEIGHT_LOW) >
         PA_FLYING_HEIGHT_THRESHOLD)
      {
        vz= PA_FLYING_HEIGHT_LOW > m_current_height_from_guidance ?
                PA_FLYING_Z_VELOCITY :
                -PA_FLYING_Z_VELOCITY;
      }
      else
        vz= 0;
      break;
    }

    case PA_START_Q:
    {
      if(fabs(m_current_height_from_guidance - PA_FLYING_HEIGHT_LOW) >
         PA_FLYING_HEIGHT_THRESHOLD)
      {
        vz= PA_FLYING_HEIGHT_LOW > m_current_height_from_guidance ?
                PA_FLYING_Z_VELOCITY :
                -PA_FLYING_Z_VELOCITY;
      }
      else
        vz= 0;
      break;
    }

    case PA_BASE_3:
    {
      /*higher at direct line*/
      if(fabs(m_line_normal[0] > 0.9))
      {
        if(fabs(m_current_height_from_guidance - PA_FLYING_HEIGHT) >
           PA_FLYING_HEIGHT_THRESHOLD)
        {
          vz= PA_FLYING_HEIGHT > m_current_height_from_guidance ?
                  PA_FLYING_Z_VELOCITY :
                  -PA_FLYING_Z_VELOCITY;
        }
        else
          vz= 0;
      }
      /*lower at bridge*/
      else
      {
        if(fabs(m_current_height_from_guidance - PA_FLYING_HEIGHT_LOW) >
           PA_FLYING_HEIGHT_THRESHOLD)
        {
          vz= PA_FLYING_HEIGHT_LOW > m_current_height_from_guidance ?
                  PA_FLYING_Z_VELOCITY :
                  -PA_FLYING_Z_VELOCITY;
        }
        else
          vz= 0;
      }
      break;
    }

    default:
    {
      if(fabs(m_current_height_from_guidance - PA_FLYING_HEIGHT) >
         PA_FLYING_HEIGHT_THRESHOLD)
      {
        vz= PA_FLYING_HEIGHT > m_current_height_from_guidance ?
                PA_FLYING_Z_VELOCITY :
                -PA_FLYING_Z_VELOCITY;
      }
      else
        vz= 0;
      break;
    }
  }
}

void RMChallengeFSM::updateVisionTask()
{
  if(m_current_takeoff_point_id == PA_PILLAR_1 ||
     m_current_takeoff_point_id == PA_PILLAR_2 ||
     m_current_takeoff_point_id == PA_PILLAR_3 ||
     m_current_takeoff_point_id == PA_PILLAR_4 ||
     m_current_takeoff_point_id == PA_START_Q)
  {
    publishPillarChange("pause");
    publishLineChange("resume");
    publishBaseChange("resume");
  }
  else if(m_current_takeoff_point_id == PA_BASE_1 ||
          m_current_takeoff_point_id == PA_BASE_2 ||
          m_current_takeoff_point_id == PA_BASE_3 ||
          m_current_takeoff_point_id == PA_START ||
          m_current_takeoff_point_id == PA_BASE_Q)
  {
    publishPillarChange("resume");
    publishLineChange("resume");
    publishBaseChange("pause");
  }
}

void RMChallengeFSM::updateTPosition()
{
  m_T_position_x= m_real_position[0];
}

bool RMChallengeFSM::forwardFarEnough()
{
  if((m_real_position[0] - m_T_position_x) > PA_FORWARD_THRESHOLD)
    return true;
  else
    return false;
}

bool RMChallengeFSM::isQulifying()
{
  if(m_current_takeoff_point_id == PA_BASE_Q)
    return true;
  else
    return false;
}

bool RMChallengeFSM::backwardFarEnough()
{
  if((m_T_position_x - m_real_position[0]) > PA_FORWARD_THRESHOLD)
    return true;
  else
    return false;
}

bool RMChallengeFSM::nextTargetIsBase()
{
  if(m_current_takeoff_point_id == PA_PILLAR_3)
    return true;
  else
    return false;
}
