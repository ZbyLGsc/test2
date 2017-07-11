// compile on different computers
#define ZBY_PC 1
#define MANIFOLD 2
#define CURRENT_COMPUTER ZBY_PC
// #define CURRENT_COMPUTER MANIFOLD

#define TAKEOFF_POINT_NUMBER 7
// parameters of uav
#define PA_DEGREE_TO_RADIAN (3.1415926 / 180.0)
#define PA_COORDINATE_TRANSFORM_DEGREE (-90)
#define PA_COORDINATE_TRANSFORM_ANGLE                                \
  PA_COORDINATE_TRANSFORM_DEGREE *PA_DEGREE_TO_RADIAN

#define PA_TAKEOFF_TIME 8
#define PA_TAKEOFF_HEIGHT_THRESHOLD 0.1
#define PA_TAKEOFF_POSITION_ERROR 2
#define PA_BASE_HEIGHT_THRESHOLD 0.2

#define PA_SETPOINT_POSITION_ERROR 1
#define PA_GRASPPER_CONTROL_TIME 6
#define PA_GO_UP_VELOCITY 0.2

#define PA_FLYING_HEIGHT 2.0
#define PA_FLYING_HEIGHT_THRESHOLD 0.2
#define PA_FLYING_Z_VELOCITY 0.15

#define PA_LAND_COUNT 3
#define PA_LAND_HEIGHT 1.05
#define PA_LAND_HEIGHT_THRESHOLD 0.05
#define PA_LAND_POSITION_THRESHOLD_LOW 0.03
#define PA_LAND_POSITION_THRESHOLD_HIGH 0.3
#define PA_V_MIN_HIGH 0.12
#define PA_V_MIN_LOW 0.036
#define PA_LAND_Z_VELOCITY 0.15
#define PA_LAND_TRIANGLE_VELOCITY_HIGH 0.15
#define PA_LAND_TRIANGLE_VELOCITY_LOW 0.07
#define PA_KP_BASE 0.4
#define PA_KP_PILLAR_HIGH 0.3
#define PA_KP_PILLAR_LOW 0.3

#define PA_KN 0.08
#define PA_KT 0.35

#define PA_YAW_RATE 10
#define PA_ANGLE_WITH_DIRECT_LINE_THRESHOLD 25
#define PA_ANGLE_THRESHOLD 10

#define PA_CAMERA_DISPLACE 0.15

#include <sstream>
#include <ros/assert.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
// C++标准库
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <math.h>
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
    TAKE_OFF,
    GO_UP,
    GO_TO_SETPOINT,
    IDLE,
    TRACK_LINE,
    LAND,
    CONTROL_GRASPPER,
    GO_TO_LAND_POINT,
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
  };
  enum LINE_TYPE
  {
    YELLOW_LINE,
    VIRTUAL_LINE,
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
  float m_goal_height[TAKEOFF_POINT_NUMBER];
  float m_takeoff_points[TAKEOFF_POINT_NUMBER + 1][2];
  float m_setpoints[TAKEOFF_POINT_NUMBER + 1][2];

  /**subscribe from dji's nodes*/
  UAV_STATE m_uav_state;
  float m_current_height_from_guidance;
  float m_current_position_from_guidance[2];  // initial
  float m_guidance_bias[2];

  /**subscribe from vision node about circle and triangle*/
  bool m_discover_pillar_circle;
  float m_landpoint_position_error[2];
  float m_current_height_from_vision;
  int m_pillar_triangle[4];

  /**subscribe from  vision node about base*/
  bool m_discover_base;

  /**subscribe from vision node about detectLine*/
  float m_distance_to_line[2];
  float m_line_normal[2];
  LAND_POINT_TYPE m_land_point_type;
  PREPARE_TO_LAND_TYPE m_prepare_to_land_type;  // initial
  int m_land_counter;                           // initial
  GRASPPER_STATE m_graspper_state= GRASPPER_CLOSE;
  int m_graspper_control_time= 0;     // initial
  int m_current_takeoff_point_id= 0;  // initial
  ros::Time m_takeoff_time;

  ros::Publisher m_velocity_pub;
  ros::Publisher m_position_pub;

public:
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
  bool finishGraspperTask();                   // tested

  /**uav control method*/
  void droneTakeoff();
  void droneLand();
  void controlDroneVelocity(float x, float y, float z, float yaw);
  void controlGraspper();    // tested
  void openGraspper();       // tested
  void closeGraspper();      // tested
  void updateTakeoffTime();  // tested
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
  void calculateYawRate(float &yaw);  // tested,confirm yaw
                                      // direction
  void transformCoordinate(float phi, float &x, float &y);
  void unitifyVector(float &x, float &y);                 // tested
  void navigateByTriangle(float &x, float &y, float &z);  // tested
  void navigateByCircle(float &x, float &y, float &z);    // tested

public:
  /**update from dji's nodes*/
  void setDroneState(int state);
  void setHeightFromGuidance(float height);
  void setPositionFromGuidance(float x, float y);  //考虑漂移，m_bias
  /**update from topic about circle and triangle*/
  void setCircleVariables(bool is_circle_found,
                          float position_error[2], float height);
  void setTriangleVariables(int pillar_triangle[4]);
  /**update from topic about base */
  void setBaseVariables(bool is_base_found, float position_error[2]);
  /**update from topic about detectLine*/
  void setLineVariables(float distance_to_line[2],
                        float line_normal[2]);
};
