/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */



#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//for serial port communication
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "std_msgs/String.h"
#include <sstream>

using namespace std;
using namespace DJI::onboardSDK;
using namespace boost::asio;


//! Function Prototypes for Mobile command callbacks - Core Functions
void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);
//! Function Prototypes for Mobile command callbacks - Custom Missions
void DrawCircleDemoMobileCallback(DJIDrone *drone);
void DrawSquareDemoMobileCallback(DJIDrone *drone);
void GimbalControlDemoMobileCallback(DJIDrone *drone);
void AttitudeControlDemoMobileCallback(DJIDrone *drone);
void LocalNavigationTestMobileCallback(DJIDrone *drone);
void GlobalNavigationTestMobileCallback(DJIDrone *drone);
void WaypointNavigationTestMobileCallback(DJIDrone *drone);
void VirtuaRCTestMobileCallback(DJIDrone *drone);

//! For LAS logging
void StartMapLASLoggingMobileCallback(DJIDrone *drone);
void StopMapLASLoggingMobileCallback(DJIDrone *drone);
void StartCollisionAvoidanceCallback(DJIDrone *drone);
void StopCollisionAvoidanceCallback(DJIDrone *drone);

//vision group defined function
void initDrone(DJIDrone *drone,uint8_t &userData);//initial a dji drone by setting callback function
void rcdataCallback(const dji_sdk::RCChannels rc_channels);
void getmessage(const std_msgs::String::ConstPtr& msg);
void flySquare(DJIDrone *drone);//control the drone to fly a trajectory of square
void flyUpDown(DJIDrone *drone);//control the drone to fly up and down
void positionControl(DJIDrone *drone, double x, double y, double z);//relative position
void velocityControl(DJIDrone *drone, double x, double y, double z);
void altitudeControl(DJIDrone *drone,double z);//absolute height
void missionUp(); //F mode and gear up
//vision group global variable
DJIDrone* drone;
dji_sdk::RCChannels rc_data;
bool channelChanged=false;
int previousMode=-8000;
int previousGear=-10000;
//information from vision
double real_dis_x = 1.0;
double real_dis_y = 1.0;
double real_height = 1.5;
double target_height=1.5;

bool circleFound;
int tri[4];
int triRight;
int triUp;
int triLeft;
int triDown;
bool preCircleFound=false;
int preTri[4]={0,0,0,0};
/****************************************************************************************
*
*main function
*
*****************************************************************************************/

int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;

    int yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    //virtual RC test data
	uint32_t virtual_rc_data[16];

	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
    ros::init(argc, argv, "rc_auto_switch");
    ROS_INFO("sdk_service_client_test");


    //initialize serial port
    io_service iosev;
    serial_port serialPort(iosev,"/dev/ttyTHS0");
    ROS_INFO("HELLO SERIAL");
    serialPort.set_option(serial_port::baud_rate(9600));
    serialPort.set_option(serial_port::flow_control(serial_port::flow_control::none));
    serialPort.set_option(serial_port::parity(serial_port::parity::none));
    serialPort.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    serialPort.set_option(serial_port::character_size(8));

	//initialize node handle and subscriber
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);
    uint8_t userData = 0;
    initDrone(drone,userData);
    ros::Subscriber rc_channels_sub = nh.subscribe("dji_sdk/rc_channels", 10,rcdataCallback);
    ros::Subscriber sub = nh.subscribe("real_distance", 1, getmessage);
    ros::spinOnce();
    ros::Rate r(50);

    //main loop
    while(ros::ok())
    {
        ROS_INFO("running main loop");
    //read rc_data from callback
        ros::spinOnce();
        if((int(rc_data.mode)!=previousMode)||(int(rc_data.gear)!=previousGear))
            channelChanged=true;
        else
            channelChanged=false;
        previousGear=int(rc_data.gear);
        previousMode=int(rc_data.mode);

	//switch according to rc_channels data
        if(fabs(rc_data.mode-8000)<0.000001)//api control,F
        {
            drone->request_sdk_permission_control();
            while(fabs(rc_data.mode-8000)<0.000001)
            {
                if(fabs(rc_data.gear+10000)<0.000001)//gear up
                {
                    cout<<"F mode,gear up"<<endl;
                    missionUp();

                }
                else if(fabs(rc_data.gear+4545)<0.000001)//gear down
                {
                    cout<<"F mode,gear down,testing position control"<<endl;
                    while(fabs(rc_data.gear+4545)<0.000001&&fabs(rc_data.mode-8000)<0.000001)
                    {
                        cout<<"F mode,gear down,testing position control"<<endl;
//                        for(int i=0;i<100;i++)
//                            positionControl(drone,0,0,1.0);
//                        positionControl(drone,2,0,0);
//                        sleep(6);
//                        positionControl(drone,-2,0,0);
//                        sleep(6);
//                        positionControl(drone,0,2,0);
//                        sleep(2);
//                        positionControl(drone,0,-2,0);
//                        sleep(2);
//                        positionControl(drone,0,0,2);
//                        sleep(4);
//                        positionControl(drone,0,0,-2);
//                        sleep(4);
//                        positionControl(drone,0,0,4);
//                        sleep(4);
//                        positionControl(drone,0,0,-4);
//                        sleep(4);
//                        positionControl(drone,0,0,4);
//                        sleep(4);
//                        positionControl(drone,0,0,4);
//                        sleep(4);
//                        positionControl(drone,0,0,-3);
//                        sleep(4);
//                        positionControl(drone,0,0,-3);
//                        sleep(4);
//                        positionControl(drone,0,0,15);
//                        sleep(4);
//                        positionControl(drone,0,0,-15);
//                        sleep(4);
//                        positionControl(drone,0,0,20);
//                        sleep(4);
//                        positionControl(drone,0,0,-20);
//                        sleep(4);
//                            velocityControl(drone,0,0,1.0);
//                            sleep(6);
//                            positionControl(drone,0,0,-1);
//                            sleep(5);
//                            velocityControl(drone,0,0,0);
//                            sleep(5);
                        ros::spinOnce();
                    }
                }
                //flying for a while
                for(int wait=0;wait<6;wait++)
                {
                    ros::spinOnce();
                    sleep(1);
                    cout<<"waiting for state change"<<endl;
                }
            }
            sleep(1);
            drone->release_sdk_permission_control();
        }
        else if(fabs(rc_data.mode)<0.000001)//att control,A
        {
            cout<<"A mode -----------"<<endl;
        }
        else if(fabs(rc_data.mode+8000)<0.000001)//pos control,P
        {
            if(fabs(rc_data.gear+10000)<0.000001)//gear up
            {
                cout<<"P mode,gear up"<<endl;
                if(channelChanged)
                {
                    write(serialPort,boost::asio::buffer("e",1));
                    cout<<"send command to DCCduino"<<endl;
                }
            }
            else if(fabs(rc_data.gear+4545)<0.000001)//gear down
            {
                cout<<"P mode,gear down"<<endl;
                if(channelChanged)
                {
                    cout<<"send command to DCCduino"<<endl;
                    write(serialPort,boost::asio::buffer("d",1));
                }
            }

        }
        main_operate_code = -1;
        r.sleep();
    }
    return 0;
}
/****************************************************************************************
*
*Callback functions for Mobile Commands
*
*****************************************************************************************/
    void ObtainControlMobileCallback(DJIDrone *drone)
    {
      drone->request_sdk_permission_control();
    }

    void ReleaseControlMobileCallback(DJIDrone *drone)
    {
      drone->release_sdk_permission_control();
    }

    void TakeOffMobileCallback(DJIDrone *drone)
    {
      drone->takeoff();
    }

    void LandingMobileCallback(DJIDrone *drone)
    {
      drone->landing();
    }

    void GetSDKVersionMobileCallback(DJIDrone *drone)
    {
      drone->check_version();
    }

    void ArmMobileCallback(DJIDrone *drone)
    {
      drone->drone_arm();
    }

    void DisarmMobileCallback(DJIDrone *drone)
    {
      drone->drone_disarm();
    }

    void GoHomeMobileCallback(DJIDrone *drone)
    {
      drone->gohome();
    }

    void TakePhotoMobileCallback(DJIDrone *drone)
    {
      drone->take_picture();
    }

    void StartVideoMobileCallback(DJIDrone *drone)
    {
      drone->start_video();
    }

    void StopVideoMobileCallback(DJIDrone *drone)
    {
      drone->stop_video();
    }

    void DrawCircleDemoMobileCallback(DJIDrone *drone)
    {
        static float R = 2;
        static float V = 2;
        static float x;
        static float y;
        int circleRadius;
        int circleHeight;
        float Phi =0, circleRadiusIncrements;
        int x_center, y_center, yaw_local; 

        circleHeight = 7;
        circleRadius = 7;

        x_center = drone->local_position.x;
        y_center = drone->local_position.y;
        circleRadiusIncrements = 0.01;

        for(int j = 0; j < 1000; j ++)
        {   
            if (circleRadiusIncrements < circleRadius)
            {
                x =  x_center + circleRadiusIncrements;
                y =  y_center;
                circleRadiusIncrements = circleRadiusIncrements + 0.01;
                drone->local_position_control(x ,y ,circleHeight, 0);
                usleep(20000);
            }
                else
            {
                break;
            }
        }
        

        /* start to draw circle */
        for(int i = 0; i < 1890; i ++)
        {   
            x =  x_center + circleRadius*cos((Phi/300));
            y =  y_center + circleRadius*sin((Phi/300));
            Phi = Phi+1;
            drone->local_position_control(x ,y ,circleHeight, 0);
            usleep(20000);
        }

    }
    void DrawSquareDemoMobileCallback(DJIDrone *drone)
    {
    /*draw square sample*/
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, 3, 0, 0 );
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, 3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, -3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, -3, 0, 0);
            usleep(20000);
        }
    }

     void GimbalControlDemoMobileCallback(DJIDrone *drone)
        {
        drone->gimbal_angle_control(0, 0, 1800, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 0, -1800, 20);
        sleep(2);
        drone->gimbal_angle_control(300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(-300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 300, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, -300, 0, 20);
        sleep(2);
        drone->gimbal_speed_control(100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(-100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(0, 0, 200);
        sleep(2);
        drone->gimbal_speed_control(0, 0, -200);
        sleep(2);
        drone->gimbal_speed_control(0, 200, 0);
        sleep(2);
        drone->gimbal_speed_control(0, -200, 0);
        sleep(2);
        drone->gimbal_angle_control(0, 0, 0, 20);
        }

    void AttitudeControlDemoMobileCallback(DJIDrone *drone)
    {
        /* attitude control sample*/
        drone->takeoff();
        sleep(8);


        for(int i = 0; i < 100; i ++)
        {
            if(i < 90)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, -2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, -2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, 0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, -0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, 90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, -90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        drone->landing();

    }
    void LocalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void GlobalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void WaypointNavigationTestMobileCallback(DJIDrone *drone)
    {
        
    }
    void VirtuaRCTestMobileCallback(DJIDrone *drone)
    {
        //virtual RC test data
        uint32_t virtual_rc_data[16];
        //virtual rc test 1: arm & disarm
        drone->virtual_rc_enable();
        usleep(20000);

        virtual_rc_data[0] = 1024;  //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;  //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024+660;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;  //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1684;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for (int i = 0; i < 100; i++){
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }

        //virtual rc test 2: yaw 
        drone->virtual_rc_enable();
        virtual_rc_data[0] = 1024;      //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024-200;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;      //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1324;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for(int i = 0; i < 100; i++) {
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }
        drone->virtual_rc_disable();
    }

void StartMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("roslaunch point_cloud_las start_velodyne_and_loam.launch &");
  system("rosrun point_cloud_las write _topic:=/laser_cloud_surround _folder_path:=. &");
}

void StopMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("rosnode kill /write_LAS /scanRegistration /laserMapping /transformMaintenance /laserOdometry  &");
}

void StartCollisionAvoidanceCallback(DJIDrone *drone)
{
  uint8_t freq[16];
  freq[0] = 1;    // 0 - Timestamp
  freq[1] = 4;    // 1 - Attitude Quaterniouns
  freq[2] = 1;    // 2 - Acceleration
  freq[3] = 4;    // 3 - Velocity (Ground Frame)
  freq[4] = 4;    // 4 - Angular Velocity (Body Frame)
  freq[5] = 3;    // 5 - Position
  freq[6] = 0;    // 6 - Magnetometer
  freq[7] = 3;    // 7 - M100:RC Channels Data, A3:RTK Detailed Information
  freq[8] = 0;    // 8 - M100:Gimbal Data, A3: Magnetometer
  freq[9] = 3;    // 9 - M100:Flight Status, A3: RC Channels
  freq[10] = 0;   // 10 - M100:Battery Level, A3: Gimble Data
  freq[11] = 2;   // 11 - M100:Control Information, A3: Flight Status

  drone->set_message_frequency(freq);
  usleep(1e4);
  system("roslaunch dji_collision_avoidance from_DJI_ros_demo.launch &");
}

void StopCollisionAvoidanceCallback(DJIDrone *drone)
{
  drone->release_sdk_permission_control();
  system("rosnode kill /drone_tf_builder /dji_occupancy_grid_node /dji_collision_detection_node /collision_velodyne_nodelet_manager /manual_fly");
  usleep(1e4);
  drone->request_sdk_permission_control();
}

/****************************************************************************************
*
*vision group defined function
*
*****************************************************************************************/
void initDrone(DJIDrone *drone,uint8_t &userData)
{
	cout<<"initialize drone"<<endl;
	//! Setting functions to be called for Mobile App Commands mode 
    	drone->setObtainControlMobileCallback(ObtainControlMobileCallback, &userData);
    	drone->setReleaseControlMobileCallback(ReleaseControlMobileCallback, &userData);
    	drone->setTakeOffMobileCallback(TakeOffMobileCallback, &userData);
    	drone->setLandingMobileCallback(LandingMobileCallback, &userData);
    	drone->setGetSDKVersionMobileCallback(GetSDKVersionMobileCallback, &userData);
    	drone->setArmMobileCallback(ArmMobileCallback, &userData);
    	drone->setDisarmMobileCallback(DisarmMobileCallback, &userData);
    	drone->setGoHomeMobileCallback(GoHomeMobileCallback, &userData);
    	drone->setTakePhotoMobileCallback(TakePhotoMobileCallback, &userData);
    	drone->setStartVideoMobileCallback(StartVideoMobileCallback,&userData);
    	drone->setStopVideoMobileCallback(StopVideoMobileCallback,&userData);
    	drone->setDrawCircleDemoMobileCallback(DrawCircleDemoMobileCallback, &userData);
    	drone->setDrawSquareDemoMobileCallback(DrawSquareDemoMobileCallback, &userData);
    	drone->setGimbalControlDemoMobileCallback(GimbalControlDemoMobileCallback, &userData);
    	drone->setAttitudeControlDemoMobileCallback(AttitudeControlDemoMobileCallback, &userData);
    	drone->setLocalNavigationTestMobileCallback(LocalNavigationTestMobileCallback, &userData);
    	drone->setGlobalNavigationTestMobileCallback(GlobalNavigationTestMobileCallback, &userData);
    	drone->setWaypointNavigationTestMobileCallback(WaypointNavigationTestMobileCallback, &userData);
    	drone->setVirtuaRCTestMobileCallback(VirtuaRCTestMobileCallback, &userData);

    	drone->setStartMapLASLoggingMobileCallback(StartMapLASLoggingMobileCallback, &userData);
    	drone->setStopMapLASLoggingMobileCallback(StopMapLASLoggingMobileCallback, &userData);
    	drone->setStartCollisionAvoidanceCallback(StartCollisionAvoidanceCallback, &userData);
    	drone->setStopCollisionAvoidanceCallback(StopCollisionAvoidanceCallback, &userData);
	
}

void flySquare(DJIDrone *drone)
{
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                3, 3, 0, 0 );
        usleep(20000);
    }
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                -3, 3, 0, 0);
        usleep(20000);
    }
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                -3, -3, 0, 0);
        usleep(20000);
    }
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                3, -3, 0, 0);
        usleep(20000);positionControl(drone,0,0,0);
    }
    cout<<"fly square ----------"<<endl;
}

void flyUpDown(DJIDrone *drone)
{
    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0xA, 0, 0, 0, 90);
        else
            drone->attitude_control(0xA, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0xA, 0, 0, 0, -90);
        else
            drone->attitude_control(0xA, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);
    cout<<"fly up and down ----------"<<endl;
}

void rcdataCallback(const dji_sdk::RCChannels rc_channels)
{
	//cout<<"time stamp:"<<rc_channels.ts<<endl
	//	<<"roll:"<<rc_channels.roll<<endl
	//	<<"pithc:"<<rc_channels.pitch<<endl
	//	<<"yaw:"<<rc_channels.yaw<<endl
	//	<<"throttle:"<<rc_channels.throttle<<endl
//        cout<<"mode:"<<rc_channels.mode<<endl
//        <<"gear:"<<rc_channels.gear<<endl;
	
	rc_data=rc_channels;

}

void positionControl(DJIDrone *drone,double x,double y,double z)
{
    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_RATE,
            x, y, z, 0);
//    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION ,
//            x, y, z, 0);
//    drone->attitude_control( 0x40 ,
//            x, y, z, 0);
//    double x_center = drone->local_position.x;
//    double y_center = drone->local_position.y;
//    double z=drone->local_position.z;
//    cout<<"local position is"<<x<<" "<<y<<endl;
//    x+=x_center;
//    y+=y_center;
//    drone->local_position_control(x ,y ,z, 0);
    usleep(20000);
}

void velocityControl(DJIDrone *drone,double x,double y,double z)
{
    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY|
            Flight::YawLogic::YAW_RATE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY | Flight::YawCoordinate::YAW_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            x, y, z, 0);
//    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION ,
//            x, y, z, 0);
//    drone->attitude_control( 0x40 ,
//            x, y, z, 0);
//    double x_center = drone->local_position.x;
//    double y_center = drone->local_position.y;
//    double z=drone->local_position.z;
//    cout<<"local position is"<<x<<" "<<y<<endl;
//    x+=x_center;
//    y+=y_center;
//    drone->local_position_control(x ,y ,z, 0);
    usleep(20000);
}

void altitudeControl(DJIDrone *drone,double z)
{
//    double x = drone->local_position.x;
//    double y = drone->local_position.y;

//    drone->local_position_control(x ,y ,z, 0);
    drone->attitude_control(0x40, 0, 0, z, 0);
    usleep(20000);
}

void missionUp()
{
    cout<<"mission F up"<<endl;
    static double x_threshold=0.1,y_threshold=0.1;//position error threshold
    static double xy_threshold_high=0.05;
    static double xy_threshold_low=0.05;
    static double disp=0.15;
    static double height_threshold=0.2;
    //while(fabs(real_dis_x)>x_threshold||fabs(real_dis_y)>y_threshold)
    while(sqrt(pow(real_dis_x,2)+pow(real_dis_y,2))>xy_threshold_high||fabs(target_height-real_height)>height_threshold)
    {
//        ros::spinOnce();
        if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+10000)>0.000001)
            return;
        if(circleFound)
        {
            double disp_h=target_height-real_height;
            positionControl(drone,real_dis_x,real_dis_y,disp_h);
            ros::spinOnce();
            cout<<"high circle:"<<real_dis_x<<" "<<real_dis_y<<endl;
        }
        else
        {
            int sum=tri[0]+tri[1]+tri[2]+tri[3];
            cout<<"sum is "<<sum<<endl;
            cout<<tri[0]<<" "<<tri[1]<<" "<<tri[2]<<" "<<tri[3]<<endl;
            double move_x=0,move_y=0;
//            double disp=0.15;
            if(sum==0)//no triangle found
            {
                cout<<"nothing found!"<<endl;
            }
            else if(sum==1)//one triangle found
            {
                if(tri[0]==1)//move to left
                {
                    move_x=0;
                    move_y=-disp;
                    cout<<"move to left"<<endl;
                }
                else if(tri[1]==1)
                {
                    move_x=-disp;
                    move_y=0;
                    cout<<"move backward"<<endl;
                }
                else if(tri[2]==1)
                {
                    move_x=0;
                    move_y=disp;
                    cout<<"move to right"<<endl;
                }
                else if(tri[3]==1)
                {
                    move_x=disp;
                    move_y=0;
                    cout<<"move to forward"<<endl;
                }
            }
            else if(sum==2)//two triangles found
            {
                if(tri[0]==1)
                {
                    move_y=-disp;
                    cout<<"move to left"<<" ";
                }
                else if(tri[2]==1)
                {
                    move_y=disp;
                    cout<<"move to right"<<" ";
                }
                if(tri[1]==1)
                {
                    move_x=-disp;
                    cout<<"move backward"<<endl;
                }
                else if(tri[3]==1)
                {
                    move_x=disp;
                    cout<<"move forward"<<endl;
                }
            }
            else if(sum==3)//three triangles found
            {
                if(tri[0]==0)
                {
                    move_x=0;
                    move_y=disp;
                    cout<<"move to right"<<endl;
                }
                else if(tri[1]==0)
                {
                    move_x=disp;
                    move_y=0;
                    cout<<"move forward"<<endl;
                }
                else if(tri[2]==0)
                {
                    move_x=0;
                    move_y=-disp;
                    cout<<"move to left"<<endl;
                }
                else if(tri[3]==0)
                {
                    move_x=-disp;
                    move_y=0;
                    cout<<"move backward"<<endl;
                }
            }
            cout<<"arrive"<<endl;
            cout<<move_x<<" "<<move_y<<" "<<endl;
            positionControl(drone,move_x,move_y,0);
            cout<<"arrive2"<<endl;
            ros::spinOnce();
        }
    }
//    for(int i=0;i<400;i++)
//    {
//        altitudeControl(drone,1.3);
//        cout<<"down to 1 m"<<endl;
//    }
//    double height_threshold=0.2;
//    if(fabs(target_height-real_height)>height_threshold&&circleFound)
//    {

//    }void getmessage(const std_msgs::String::ConstPtr& msg)
//    ros::spinOnce();
//    for(int i=0;i<10;i++)
//        velocityControl(drone,0,0,0);
//    double dispz=target_height-real_height;
//    positionControl(drone,0,0,dispz);
//    sleep(1);
//    for(int i=0;i<10;i++)
//        velocityControl(drone,0,0,0);

//    while(sqrt(pow(real_dis_x,2)+pow(real_dis_y,2))>xy_threshold_low)
//    {
//        if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+10000)>0.000001)
//            return;

//        if(circleFound)
//        {
//            positionControl(drone,real_dis_x,real_dis_y,0);
//            ros::spinOnce();
//            cout<<"low circle:"<<real_dis_x<<" "<<real_dis_y<<endl;
//        }
//        else
//        {
//            int sum=tri[0]+tri[1]+tri[2]+tri[3];
//            int move_x=0,move_y=0;
//            if(sum==0)//no triangle found
//            {
//                cout<<"nothing found!"<<endl;
//            }
//            if(sum==1)//one triangle found
//            {
//                if(tri[0]==1)//move to left
//                {
//                    move_x=0;
//                    move_y=-disp;
//                    cout<<"move to left"<<endl;
//                }
//                else if(tri[1]==1)
//                {
//                    move_x=-disp;
//                    move_y=0;
//                    cout<<"move backward"<<endl;
//                }
//                else if(tri[2]==1)
//                {
//                    move_x=0;
//                    move_y=disp;
//                    cout<<"move to right"<<endl;
//                }
//                else if(tri[3]==1)
//                {
//                    move_x=disp;
//                    move_y=0;
//                    cout<<"move to forward"<<endl;
//                }
//            }
//            if(sum==2)//two triangles found
//            {
//                if(tri[0]==1)
//                {
//                    move_y=-disp;
//                    cout<<"move to left"<<" ";
//                }
//                else if(tri[2]==1)
//                {
//                    move_y=disp;
//                    cout<<"move to right"<<" ";
//                }
//                if(tri[1]==1)
//                {double disp=0.15;
//                    move_x=-disp;
//                    cout<<"move backward"<<endl;
//                }
//                else if(tri[3]==1)
//                {
//                    move_x=disp;
//                    cout<<"move forward"<<endl;
//                }
//            }
//            if(sum==3)//three triangles found
//            {
//                if(tri[0]==0)
//                {
//                    move_x=0;
//                    move_y=disp;
//                    cout<<"move to right"<<endl;
//                }
//                else if(tri[1]==0)
//                {
//                    move_x=disp;
//                    move_y=0;
//                    cout<<"move forward"<<endl;
//                }
//                else if(tri[2]==0)
//                {
//                    move_x=0;
//                    move_y=-disp;
//                    cout<<"move to left "<<endl;
//                }
//                else if(tri[3]==0)
//                {
//                    move_x=-disp;
//                    move_y=0;
//                    cout<<"move backward  "<<endl;
//                }
//            }
//            positionControl(drone,move_x,move_y,0);
//            ros::spinOnce();
//        }
//    }
    drone->landing();
}

void getmessage(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("got: [%s]", msg->data.c_str());
    std::stringstream ss(msg->data.c_str());

    double y,x,h;
//    ss >>y>>x>>tri[0]>>tri[1]>>tri[2]>>tri[3];
    ss >>triRight>>triUp>>triLeft>>triDown>>y>>x>>h;
    cout<<triRight<<" "<<triUp<<" "<<triLeft<<" "<<triDown<<endl;
    tri[0] = triRight;
    tri[1] = triUp;
    tri[2] = triLeft;
    tri[3] = triDown;
    if(fabs(x)<0.000000001&&fabs(y)<0.000000001)
        circleFound=false;
    else
    {
        circleFound=true;
        if(fabs(y)>0.000000001)
            real_dis_y=y/1000;
        if(fabs(x)>0.000000001)
            real_dis_x=x/1000;
    if(fabs(h)>0.000000001)
            real_height=h/1000;
    }
    cout<<real_dis_x<<" "<<real_dis_y<<" "<<real_height<<" "<<tri[0]<<" "<<tri[1]<<" "
       <<tri[2]<<" "<<tri[3]<<endl;
}

