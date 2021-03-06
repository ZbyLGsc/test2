// compile on different computers
#define ZBY_PC 1
#define MANIFOLD 2
//#define CURRENT_COMPUTER ZBY_PC
 #define CURRENT_COMPUTER MANIFOLD

/**parameters of uav*/
/*
start1,pillar1-4,base1-4,(formal),9
start2,base5,pillar5,(qulification),3
*/
#define TAKEOFF_POINT_NUMBER 12
#define PA_DEGREE_TO_RADIAN (3.1415926 / 180.0)
#define PA_ARENA_ANGLE_RED 15
#define PA_ARENA_ANGLE_BLUE (-160)
#define PA_COORDINATE_TRANSFORM_DEGREE PA_ARENA_ANGLE_RED
#define PA_COORDINATE_TRANSFORM_ANGLE                                          \
  PA_COORDINATE_TRANSFORM_DEGREE *PA_DEGREE_TO_RADIAN
#define PA_TAKEOFF_TIME 7
#define PA_TAKEOFF_HEIGHT 2.4
#define PA_TAKEOFF_HEIGHT_THRESHOLD 0.1
#define PA_TAKEOFF_POSITION_ERROR 2.5
#define PA_SETPOINT_POSITION_ERROR 0.5
#define PA_LANDPOINT_POSITION_ERROR 4.0
#define PA_GRASPPER_CONTROL_TIME 1
#define PA_GO_UP_VELOCITY 0.3

#define PA_BRIDGE_HEIGHT 0.8  // set to lower than real
#define PA_FLYING_HEIGHT 2.8
#define PA_FLYING_HEIGHT_LOW (PA_FLYING_HEIGHT - PA_BRIDGE_HEIGHT)
#define PA_FLYING_HEIGHT_THRESHOLD 0.2
#define PA_FLYING_Z_VELOCITY 0.2



#define PA_LAND_COUNT 1
#define PA_TIME_MIN 1.5
#define PA_TIME_MAX 5.0
#define PA_LAND_HEIGHT 1.05
#define PA_LAND_HEIGHT_FINAL 0.6
#define PA_LAND_HEIGHT_THRESHOLD 2.0
#define PA_LAND_HEIGHT_THRESHOLD_FINAL 0.2
#define PA_LAND_POSITION_THRESHOLD_HIGH 0.4
#define PA_LAND_POSITION_THRESHOLD_LOW 0.3
#define PA_LAND_POSITION_THRESHOLD_SUPER_LOW 0.065
#define PA_LAND_POSITION_THRESHOLD_SUPER_LOW_BIG 0.15
#define PA_V_MIN_HIGH 0.15
#define PA_V_MIN_LOW 0.04
#define PA_V_MIN_FINAL 0.04
#define PA_LAND_Z_VELOCITY_FINAL 0.15
#define PA_LAND_Z_VELOCITY 0.15
#define PA_LAND_TRIANGLE_VELOCITY_HIGH 0.15
#define PA_LAND_TRIANGLE_VELOCITY_LOW 0.07
#define PA_KP_BASE 0.35
#define PA_KP_PILLAR_HIGH 0.3
#define PA_KP_PILLAR_LOW 0.3

#define PA_KN 0.08
#define PA_KT 0.35
#define PA_KT_RATIO 1

#define PA_YAW_RATE 3
#define PA_ANGLE_WITH_DIRECT_LINE_THRESHOLD 25
#define PA_ANGLE_THRESHOLD 3

#define PA_CAMERA_DISPLACE 0.16
#define PA_CAMERA_F 507.75

/*formal contest takeoff point id*/
#define PA_START 0
#define PA_PILLAR_1 1
#define PA_BASE_1 2
#define PA_PILLAR_2 3
#define PA_BASE_2 4
#define PA_PILLAR_3 5
#define PA_BASE_3 6
#define PA_PILLAR_4 7
#define PA_BASE_4 8
// #define PA_T_1 9
// #define PA_T_2 10
/*qulification contest takeoff point id*/
#define PA_START_Q 9
#define PA_BASE_Q 10
#define PA_PILLAR_Q 11

#define PA_RELEASE_BALL_HEIGHT 2.0
#define PA_RELEASE_BALL_HEIGHT_THRESHOLD 0.2
#define PA_BASE_POSITION_THRESHOLD 0.15
#define PA_RELEASE_BALL_VELOCITY 0.1
#define PA_SLOW_DOWN_HEIGHT 0.5
#define PA_BASE_HEIGHT 2.7
#define PA_BASE_HEIGHT_THRESHOLD 0.2
#define PA_BASE_Z_VELOCITY 0.2
#define PA_BASE_ANGLE_THRESHOLD 5
#define PA_BASE_YAW_RATE 7
#define PA_BASE_MIN_V 0.1

#define PA_T_DISPLACE 1.6
#define PA_TARMAC_HEIGHT 1.6
#define PA_PILLAR_HEIGHT 1.5

#define PA_FORWARD_THRESHOLD 1.5

#include <geometry_msgs/Vector3Stamped.h>
#include <ros/assert.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sstream>
#include "std_msgs/String.h"
// C++标准库
#include <math.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
// boost asio
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#if CURRENT_COMPUTER == MANIFOLD
#include <dji_sdk/dji_drone.h>
using namespace DJI::onboardSDK;
#endif

using namespace std;
using namespace boost::asio;
// const definition
class RMChallengeFSM
{
public:
  enum TASK_STATE
  {
    TAKE_OFF,  // 0
    GO_UP,
    GO_TO_SETPOINT,
    IDLE,  // 3
    TRACK_LINE,
    LAND,
    GRAB_BALL,  // 6
    GO_TO_LAND_POINT,
    GO_TO_PILLAR,
    RELEASE_BALL,  // 9
    CROSS_ARENA,
    TRACK_LINE_FORWARD,
    TRACK_LINE_BACKWARD,  // 12
    FINAL,
  };
  enum GRASPPER_STATE
  {
    GRASPPER_OPEN,
    GRASPPER_CLOSE,
  };
  enum UAV_STATE
  {
    UAV_FLY,
    UAV_LAND,
  };
  enum LAND_POINT_TYPE
  {
    BASE_LAND_POINT,
    PILLAR_LAND_POINT,
  };
  enum PREPARE_TO_LAND_TYPE
  {
    PREPARE_AT_HIGH,
    PREPARE_AT_LOW,
    PREPARE_AT_SUPER_LOW,
  };
  enum LINE_TYPE
  {
    YELLOW_LINE,
    VIRTUAL_LINE_SETPOINT,
    VIRTUAL_LINE_LANDPOINT,
  };
  enum BASE_STATE
  {
    BASE_POSITION,
    BASE_ANGLE,
  };
  enum PILLAR_COLOR
  {
    PILLAR_RED,
    PILLAR_BLUE,
  };
  RMChallengeFSM()
  {
  }
  ~RMChallengeFSM();
  void run();
  void initialize(ros::NodeHandle &node_handle);
  void resetAllState();

private:
  /**serial port*/
  boost::asio::serial_port *m_serial_port;
  boost::system::error_code m_err_code;
  boost::asio::io_service m_io_service;

/**dji sdk */
#if CURRENT_COMPUTER == MANIFOLD
  DJIDrone *m_drone;
#endif

  TASK_STATE m_state;  // initial
                       /**
                       *takeoff point id,0 is start point
                       *1,2,4,5 are pillar,3, 6 are base
                       */
  float m_goal_height[TAKEOFF_POINT_NUMBER + 1];
  float m_takeoff_points[TAKEOFF_POINT_NUMBER + 1][2];
  float m_setpoints[TAKEOFF_POINT_NUMBER + 1][2];

  /**subscribe from dji's nodes*/
  UAV_STATE m_uav_state= UAV_LAND;
  float m_current_height_from_guidance;
  /**
  raw_position=real_position+bias
  bias=raw - real
  */
  float m_real_position[2];  // initial
  float m_guidance_bias[2];
  float m_raw_guidance_position[2];

  /**subscribe from vision node about circle,arc and triangle*/
  bool m_discover_pillar_circle;
  bool m_discover_pillar_arc;
  float m_circle_position_error[2];
  float m_current_height_from_circle;
  float m_arc_position_error[2];
  float m_current_height_from_arc;
  int m_pillar_triangle[4];
  PILLAR_COLOR m_first_pillar_color;

  /**subscribe from  vision node about base*/
  bool m_discover_base;
  float m_base_position_error[2];
  float m_base_angle;
  BASE_STATE m_base_state;

  /**subscribe from vision node about detectLine*/
  float m_distance_to_line[2];
  float m_line_normal[2];
  bool m_discover_T;
  bool m_already_find_T;

  /**internal variables related to uav state*/
  LAND_POINT_TYPE m_land_point_type;
  PREPARE_TO_LAND_TYPE m_prepare_to_land_type;  // initial
  int m_land_counter;                           // initial
  GRASPPER_STATE m_graspper_state= GRASPPER_CLOSE;
  int m_graspper_control_time= 0;            // initial
  int m_current_takeoff_point_id= PA_START;  // initial
  ros::Time m_takeoff_time;
  ros::Time m_checked_time;
  float m_T_position_x;

  /**publish debug message*/
  ros::Publisher m_velocity_pub;
  ros::Publisher m_position_pub;
  /**change pillar detecting color*/
  ros::Publisher m_color_change_pub;
  /**change vision task*/
  ros::Publisher m_pillar_change_pub;
  ros::Publisher m_line_change_pub;
  ros::Publisher m_base_change_pub;

private:
  /**uav state checking method*/
  void transferToTask(TASK_STATE task_state);  // tested
  bool isTakeoffTimeout();                     // tested
  bool isTakingoff();                          // tested
  bool isOnLand();                             // tested
  bool closeToGoalHeight();                    // tested
  bool farFromTakeoffPoint();                  // tested
  bool discoverLandPoint();                    /// tested
  bool discoverTriangle();                     /// tested
  bool discoverYellowLine();                   // tested
  bool closeToSetPoint();                      // tested
  bool readyToLand();                          // tested
  bool finishGrabBallTask();                   // tested
  bool isCheckedTimeSuitable();
  bool landPointIsPillar();
  bool landPointIsBase();
  bool isTheLastTravel();
  bool stillFindLandPoint();
  bool lowEnoughToReleaseBall();
  bool discoverT();                //?
  bool nextTargetIsClosePillar();  //?
  bool nextTargetIsFarPillar();    //?
  bool nextTargetIsBase();
  bool forwardFarEnough();
  bool backwardFarEnough();
  bool isQulifying();

  /**uav control method*/
  void droneTakeoff();
  void droneLand();
  void controlDroneVelocity(float x, float y, float z, float yaw);
  void grabBall();           // tested
  void openGraspper();       // tested
  void closeGraspper();      // tested
  void updateTakeoffTime();  // tested
  void updateCheckedTime();
  void droneGoUp();          // tested
  void droneGoToSetPoint();  // tested
  void droneTrackLine();     // tested
  void droneHover();         // tested
  void droneDropDown();
  void dronePrepareToLand();  // tested
  void calculateNormalVelocity(float &x, float &y,
                               LINE_TYPE lint_type);  // tested
  void calculateTangentialVelocity(float &x, float &y,
                                   LINE_TYPE lint_type);  // tested
  void calculateYawRate(float &yaw);                      // tested,confirm yaw
                                                          // direction
  void unitifyVector(float &x, float &y);  // tested
  void judgeLineDirection();
  void calculateRealPositionError(float error[2]);
  void navigateByTriangle(float &x, float &y, float &z);  // tested
  void navigateByCircle(float &x, float &y, float &z);    // tested
  void navigateByArc(float &x, float &y, float &z);
  void publishVelocity(std::string id, float x, float y, float z);
  void droneReleaseBall();
  void droneGoDownToBase();
  void droneGoToPillar();
  void updateTakeoffPointId();
  void droneUpdatePosition(int POSITION_ID= 0);

  void printStateInfo();
  void publishColorChange(std::string color);
  void publishPillarChange(std::string state);
  void publishLineChange(std::string state);
  void publishBaseChange(std::string state);
  void updatePillarColor();
  void updateVisionTask();

  void calculateZVelocity(float &vz);
  void updateTPosition();
  void navigateByQRCode(float &x, float &y, float &z, float &yaw);
  void publishPosition();

public:
  /**update from dji's nodes*/
  void setDroneState(int state);
  void setHeightFromGuidance(float height);
  void setPositionFromGuidance(float x, float y);  //考虑漂移，m_bias
  /**update from topic about circle and triangle*/
  void setCircleVariables(bool is_circle_found, float position_error[2],
                          float height);
  void setTriangleVariables(int pillar_triangle[4]);
  void setArcVariables(bool is_arc_found, float position_error[2]);
  /**update from topic about base */
  void setBaseVariables(bool is_base_found, float position_error[2],
                        float base_angle);
  /**update from topic about detectLine*/
  void setLineVariables(bool is_T_found, float distance_to_line[2],
                        float line_normal[2]);

  void setFirstPillarColor(PILLAR_COLOR color);
  void transformCoordinate(float phi, float &x, float &y);
};
