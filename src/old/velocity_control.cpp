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
void getLanding(const std_msgs::String::ConstPtr& msg);
void getHero(const std_msgs::String::ConstPtr& msg);
void getRed(const std_msgs::String::ConstPtr& msg);
void getFlightStatus(const std_msgs::UInt8::ConstPtr& msg);
void getLeftRight(const std_msgs::String::ConstPtr& msg);
void flySquare(DJIDrone *drone);//control the drone to fly a trajectory of square
void flyUpDown(DJIDrone *drone);//control the drone to fly up and down
void velocityControl(DJIDrone *drone, double x, double y, double z);
void yawControl(DJIDrone *drone,double z);
void gimbalControl(DJIDrone *drone,int roll, int pitch, int yaw, int duration);
void missionUp(); //F mode and gear up
void missionDown(); //F mode and gear up
void testVelocityControl();//F mode and gear down
void runEngineeringContest();
//vision group global variable
DJIDrone* drone;
dji_sdk::RCChannels rc_data;
bool channelChanged=false;
int previousMode=-8000;
int previousGear=-10000;

int counterFU=0;
int counterFD=0;
int counterAU=0;
int counterAD=0;
//information from vision
double real_dis_x = 1.0;
double real_dis_y = 1.0;
double real_height = 1.0;
double real_z=0;
bool isVisionRunning=false;

//PD's D control
double ex_real_dis_x = 1.0;
double ex_real_dis_y = 1.0;
double ex_sign = 0;

//autonomous landing
bool circleFound;
int tri[4];
int triRight;
int triUp;
int triLeft;
int triDown;
bool preCircleFound=false;
int preTri[4]={0,0,0,0};
double high_target_height=1.5;
double low_target_height=1.05;
int flightStatus=0;

//hero
bool heroFound;
bool prehero;
double heroTargetH=0.9;
double heroYaw=0.0;
int heroState=1;//1,2,3

//engineering technique contest
bool redLineFound=false;
bool preRed=false;
int leftRight=0;
int perlr=0;

//serial port
serial_port *serialPort;
boost::system::error_code errc;
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
//    serial_port serialPort(iosev,"/dev/ttyTHS0");
    ROS_INFO("HELLO SERIAL");
    serialPort= new serial_port(iosev);
    serialPort->open("/dev/ttyTHS0",errc);
    serialPort->set_option(serial_port::baud_rate(9600),errc);
    serialPort->set_option(serial_port::flow_control(serial_port::flow_control::none),errc);
    serialPort->set_option(serial_port::parity(serial_port::parity::none),errc);
    serialPort->set_option(serial_port::stop_bits(serial_port::stop_bits::one),errc);
    serialPort->set_option(serial_port::character_size(8),errc);

	//initialize node handle and subscriber
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);
    uint8_t userData = 0;
    initDrone(drone,userData);

    ros::Publisher flag_pub = nh.advertise<std_msgs::String>("tpp/flag", 10);
    ros::Subscriber rc_channels_sub = nh.subscribe("dji_sdk/rc_channels", 10,rcdataCallback);
    ros::Subscriber landing_sub = nh.subscribe("tpp/landing", 1, getLanding);
    ros::Subscriber hero_sub = nh.subscribe("tpp/hero", 1, getHero);
    ros::Subscriber red_sub = nh.subscribe("tpp/red", 1, getRed);
    ros::Subscriber leftright_sub = nh.subscribe("tpp/leftright", 1, getLeftRight);
    ros::Subscriber flightstatus_sub = nh.subscribe("/dji_sdk/flight_status", 1, getFlightStatus);
    ros::spinOnce();
    ros::Rate r(100);

    //test gimbal
    cout<<"test gimbal"<<endl;
    gimbalControl(drone,0,-900,0,10);

    //signal about all program running
    cout<<"waiting for program running"<<endl;
    while((!isVisionRunning||fabs(rc_data.gear+10000)>0.000001)&&ros::ok())
    {
        ros::spinOnce();
        for(int i=0;i<10;i++)
                write(*serialPort,boost::asio::buffer("c"),errc);
        usleep(1000000*0.3);
        for(int i=0;i<10;i++)
                write(*serialPort,boost::asio::buffer("d"),errc);
        usleep(1000000*0.3);
        for(int i=0;i<10;i++)
                write(*serialPort,boost::asio::buffer("e"),errc);
        usleep(1000000*0.3);
        cout<<"waiting for program running"<<endl;
    }
    for(int i=0;i<10;i++)
            write(*serialPort,boost::asio::buffer("c"),errc);
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
                ros::spinOnce();
                if(fabs(rc_data.gear+10000)<0.000001)//gear up
                {
                    counterFU++;
                    if(counterFU>200)
                    {
                        cout<<"F mode,gear up"<<endl;
                        counterAD=0;
                        counterAU=0;
                        counterFD=0;
                        counterFU=0;

                        std_msgs::String msg;
                        std::stringstream ss;
                        ss <<'l';
                        msg.data = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
                        for(int i=0;i<10;i++)
                            flag_pub.publish(msg);

                        missionUp();
//                        runEngineeringContest();
                    }
                }
                else if(fabs(rc_data.gear+4545)<0.000001)//gear down
                {

                    counterFD++;
                    if(counterFD>200)
                    {
                        cout<<"F mode,gear down,testing position control"<<endl;
                        counterAD=0;
                        counterAU=0;
                        counterFD=0;
                        counterFU=0;

                        std_msgs::String msg;
                        std::stringstream ss;
                        ss <<'h';
                        msg.data = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
                        for(int i=0;i<10;i++)
                            flag_pub.publish(msg);

                        missionDown();
                        //testVelocityControl();
                    }

                }
                r.sleep();
//                //flying for a while
//                for(int wait=0;wait<2;wait++)
//                {
//                    ros::spinOnce();
//                    sleep(1);
//                    cout<<"waiting for state change"<<endl;
//                }


            }
            sleep(1);
            drone->release_sdk_permission_control();
        }
        else if(fabs(rc_data.mode)<0.000001)//att control,A
        {
//            cout<<"A mode -----------"<<endl;
            if(fabs(rc_data.gear+10000)<0.000001)//gear up
            {
                cout<<"A mode up-----------"<<endl;
                //waiting for sometime
                counterAU++;
                if(counterAU>0)
                {
                    cout<<"now begin to read color"<<endl;
                    //read color status from arduino
                    char buf[5];
                    serialPort->read_some(boost::asio::buffer(buf),errc);
                    cout<<"now serial read begin"<<endl;
                    if(buf[0]=='r'&&buf[1]=='r'&&buf[2]=='r')
                    {
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss <<'r';
                        msg.data = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
                        for(int i=0;i<10;i++)
                            flag_pub.publish(msg);
                        cout<<"current detecting color is red red red"<<endl;
                        write(*serialPort,boost::asio::buffer("c"),errc);
                        sleep(1);
                        write(*serialPort,boost::asio::buffer("e"),errc);
                        sleep(1);
                        write(*serialPort,boost::asio::buffer("c"),errc);
                        sleep(1);                      
                    }
                    else if(buf[0]=='b'&&buf[1]=='b'&&buf[2]=='b')
                    {
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss <<'b';
                        msg.data = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
                        for(int i=0;i<10;i++)
                            flag_pub.publish(msg);

                        cout<<"current detecting color is blue blue blue"<<endl;
                        write(*serialPort,boost::asio::buffer("d"),errc);
                        sleep(1);
                        write(*serialPort,boost::asio::buffer("e"),errc);
                        sleep(1);
                        write(*serialPort,boost::asio::buffer("d"),errc);
                        sleep(1);
                        write(*serialPort,boost::asio::buffer("c"),errc);
                    }
                    else
                    {
                        cout<<"something go wrong!!"<<endl;
                    }
                    counterAU=0;
                    counterAD=0;
                    counterFD=0;
                    counterFU=0;
                }
            }
            else if(fabs(rc_data.gear+4545)<0.000001)//gear down
            {
                cout<<"A mode down-----------"<<endl;
                counterAD++;
                if(counterAD>0)
                {
                    counterAU=0;
                    counterAD=0;
                    counterFD=0;
                    counterFU=0;
                }
            }

        }
        else if(fabs(rc_data.mode+8000)<0.000001)//pos control,P
        {
            if(fabs(rc_data.gear+10000)<0.000001)//gear up
            {
                cout<<"P mode,gear up"<<endl;
                counterAD=0;
                counterAU=0;
                counterFD=0;
                counterFU=0;
                //control gimbal
                if(flightStatus==3)
                    gimbalControl(drone,0,-900,0,10);
                else if(flightStatus==1)
                    gimbalControl(drone,0,-200,-1750,10);
                if(channelChanged)
                {
                    for(int i=0;i<10;i++)
                    write(*serialPort,boost::asio::buffer("a"),errc);
                    cout<<"send command to DCCduino,close"<<endl;
                }
            }
            else if(fabs(rc_data.gear+4545)<0.000001)//gear down
            {
                cout<<"P mode,gear down"<<endl;
                counterFU=0;
                counterFD=0;
                counterAD=0;
                counterAU=0;
                //control gimbal
                if(flightStatus==3)
                    gimbalControl(drone,0,-900,0,10);
                else if(flightStatus==1)
                    gimbalControl(drone,0,-200,-1750,10);
                if(channelChanged)
                {
                    cout<<"send command to DCCduino,open open"<<endl;
                    for(int i=0;i<10;i++)
                    write(*serialPort,boost::asio::buffer("b"),errc);
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
        usleep(20000);
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

void velocityControl(DJIDrone *drone,double x,double y,double z)
{
    drone->attitude_control( 0x4B,
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

void yawControl(DJIDrone *drone,double z)
{
    drone->attitude_control( 0x4B,
            0, 0, 0,z);
    usleep(20000);
}
void missionUp()
{
    cout<<"mission F up"<<endl;
    static double x_threshold=0.1,y_threshold=0.1;//position error threshold
    static double xy_threshold_high=0.3;
    static double xy_threshold_low=0.03;
    static double disp=0.200;
    static double height_threshold=0.05;
    //while(fabs(real_dis_x)>x_threshold||fabs(real_dis_y)>y_threshold)   
    //**************************************high height step*************************************//
    int counter_h=0;
    while(sqrt(pow(real_dis_x,2)+pow(real_dis_y,2))>xy_threshold_high||
          fabs(low_target_height-real_height)>height_threshold||counter_h<2)
    {
//        ros::spinOnce();
        if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+10000)>0.000001)
            return;
        if(circleFound)
        {
            //****************** pd control ****************//
//                if(ex_sign == 0)
//                {
//                double disp_h=high_target_height-real_height;
//                velocityControl(drone,0.8*real_dis_x,0.8*real_dis_y,disp_h);
//                ex_real_dis_x = real_dis_x;
//                ex_real_dis_y = real_dis_y;
//                ros::spinOnce();
//                cout<<"high circle:"<<real_dis_x<<" "<<real_dis_y<<endl;
//                ex_sign = 1;
//                }
//                else
//                {
//                    double disp_h=high_target_height-real_height;

//                    velocityControl(drone,0.7*real_dis_x+0.5*(real_dis_x-ex_real_dis_x),0.7*real_dis_y+0.5*(real_dis_y-ex_real_dis_y),disp_h);
//                    ex_real_dis_x = real_dis_x;
//                    ex_real_dis_y = real_dis_y;
//                    ros::spinOnce();
//                    cout<<"high circle:"<<real_dis_x<<" "<<real_dis_y<<endl;
//                }
            //***********************************************//
//            double disp_h=high_target_height-real_height;
            if(sqrt(pow(real_dis_x,2)+pow(real_dis_y,2))>xy_threshold_high)
            {
                double vx=real_dis_x;
                double vy=real_dis_y;
                double vmin=0.4;
                if(fabs(vx)<vmin)
                    vx=fabs(vx)/(vx+0.000000001)*vmin;
                if(fabs(vy)<vmin)
                    vy=fabs(vy)/(vy+0.000000001)*vmin;
                
		 velocityControl(drone,0.3*vx,0.3*vy,0);

                ros::spinOnce();
                cout<<"high circle:"<<real_dis_x<<" "<<real_dis_y<<endl;
            }
            else
            {
                cout<<"control height"<<endl;
                double disp_l=low_target_height-real_height;
                double vh=disp_l*0.5;
                if(fabs(vh)<0.15)
                    vh=fabs(vh)/(vh+0.0000000001)*0.15;
                double sign=disp_l>0?1.0:-1.0;
                velocityControl(drone,0*real_dis_x,0*real_dis_y,vh);
                counter_h++;
                ros::spinOnce();
            }
        }
        else
        {
            ex_sign = 0;
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
            velocityControl(drone,move_x,move_y,0);
            cout<<"arrive2"<<endl;
            ros::spinOnce();
        }
    }

    //*****************************************************************//
    for(int i=0;i<100;i++)
        velocityControl(drone,0,0,0);

    int counter=0;
    while(counter<4)
    {
        //**************************low height step***********************//
        while(sqrt(pow(real_dis_x,2)+pow(real_dis_y,2))>xy_threshold_low)
        {
    //        ros::spinOnce();
            if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+10000)>0.000001)
                return;
            if(circleFound)
            {
                double disp_l=low_target_height-real_height;
                double vx=real_dis_x;
                double vy=real_dis_y;
                double vmin=0.12;
                if(fabs(vx)<vmin)
                    vx=fabs(vx)/(vx+0.000000001)*vmin;
                if(fabs(vy)<vmin)
                    vy=fabs(vy)/(vy+0.000000001)*vmin;
                        velocityControl(drone,0.3*vx,0.3*vy,0);

                ros::spinOnce();
                cout<<"low circle:"<<real_dis_x<<" "<<real_dis_y<<endl;
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
                velocityControl(drone,move_x,move_y,0);
                cout<<"arrive2"<<endl;
                ros::spinOnce();
            }
        }
        //*******************************************************************//
        counter++;
        for(int i=0;i<100;i++)
            velocityControl(drone,0,0,0);
    }
    for(int i=0;i<50;i++)
        velocityControl(drone,0,0,0);
    for(int i=0;i<20;i++)
            write(*serialPort,boost::asio::buffer("b"),errc);
    //drone->landing();
    for(int i=0;i<200;i++)               //fast landing
        velocityControl(drone,0,0,-1.2);
    drone->landing();
    gimbalControl(drone,0,-200,-1750,10);
//    for(int i=0;i<10;i++)
//            write(*serialPort,boost::asio::buffer("b"),errc);
}

void missionDown()
{
    cout<<"mission F down"<<endl; 
    ros::spinOnce();
    //move close to hero
    static double xy_initial_threshold=0.3;
    while(sqrt(pow(real_dis_x,2)+pow(real_dis_y,2))>xy_initial_threshold)
    {
        heroState=1;
        if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+4545)>0.000001)
            return;
        if(heroFound)
        {
            double vx=real_dis_x;
            double vy=real_dis_y;
            double vmin=0.4;
            if(fabs(vx)<vmin)
                vx=fabs(vx)/(vx+0.000000001)*vmin;
            if(fabs(vy)<vmin)
                vy=fabs(vy)/(vy+0.000000001)*vmin;
            velocityControl(drone,0.3*vx,0.3*vy,0);
            cout<<"go to hero initially"<<endl;
            cout<<"x y is:"<<real_dis_x<<" "<<real_dis_y<<" "<<endl;
        }
        else
        {
            velocityControl(drone,0,0,0);
            cout<<"looking for hero and go to it"<<endl;
        }
        ros::spinOnce();
    }
    for(int i=0;i<50;i++)
        velocityControl(drone,0,0,0);
    //turn directon to hero  
    while(fabs(heroYaw)>10)
    {
        heroState=2;
        if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+4545)>0.000001)
            return;
        if(heroFound)
        {
            //>0 counter clockwise <0 clockwise
            double yaw=heroYaw>0?-12:12;
            yawControl(drone,yaw);
            //adjust angle
            cout<<"turn direction"<<yaw<<endl;
            cout<<"x y is:"<<real_dis_x<<" "<<real_dis_y<<" "<<endl;
        }
        else
        {
            velocityControl(drone,0,0,0);
            cout<<"looking for hero and turn to it"<<endl;
        }
        ros::spinOnce();
    }
    for(int i=0;i<50;i++)
        velocityControl(drone,0,0,0);
    //adjust position and height
    static double xy_threshold=0.2;
    static double height_threshold=0.15;
    real_dis_x=real_dis_x-0.025;
    real_dis_y=real_dis_y+0.11;
    while(sqrt(pow(real_dis_x,2)+pow(real_dis_y,2))>xy_threshold||fabs(heroTargetH-real_height)>height_threshold)
    {
        heroState=3;
        if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+4545)>0.000001)
            return;
        if(heroFound)
        {
            if(sqrt(pow(real_dis_x,2)+pow(real_dis_y,2))>xy_threshold)
            {
                cout<<"go to hero"<<endl;
                double vx=real_dis_x;
                double vy=real_dis_y;
                double vmin=0.2;
                if(fabs(vx)<vmin)
                    vx=fabs(vx)/(vx+0.000000001)*vmin;
                if(fabs(vy)<vmin)
                    vy=fabs(vy)/(vy+0.000000001)*vmin;
                        velocityControl(drone,0.4*vx,0.4*vy,0);
            }
            else if(fabs(heroTargetH-real_height)>height_threshold)
            {
                double disp_l=heroTargetH-real_height;
                double vh=disp_l*0.5;
                if(fabs(vh)<0.15)
                    vh=fabs(vh)/(vh+0.0000000001)*0.15;
                double sign=disp_l>0?1.0:-1.0;
                velocityControl(drone,0*real_dis_x,0*real_dis_y,sign*0.22);
            }
        }
        else
        {
            velocityControl(drone,0,0,0);
            cout<<"looking for hero"<<endl;
        }
        ros::spinOnce();
    }
    //hover and release ball
    for(int i=0;i<100;i++)
        velocityControl(drone,0,0,0);
    for(int i=0;i<10;i++)
            write(*serialPort,boost::asio::buffer("b"),errc);
    while(fabs(rc_data.mode-8000)<0.000001&&fabs(rc_data.gear+4545)<0.000001)
    {
        ros::spinOnce();
        cout<<"waiting in F down"<<endl;
    }
    for(int i=0;i<10;i++)
            write(*serialPort,boost::asio::buffer("a"),errc);
    heroState=1;
}

void getLanding(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("got: [%s]", msg->data.c_str());
    std::stringstream ss(msg->data.c_str());
    isVisionRunning=true;
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
            real_dis_x=(x/1000)-0.15;
            //real_dis_x=(x/1000)-0.13;
    if(fabs(h)>0.000000001)
            real_height=h/1000;
    }
    cout<<real_dis_x<<" "<<real_dis_y<<" "<<real_height<<" "<<tri[0]<<" "<<tri[1]<<" "
       <<tri[2]<<" "<<tri[3]<<endl;
    //write message to arduino
    bool stateChanged=false;
    int cursum=tri[0]+tri[1]+tri[2]+tri[3];
    int presum=preTri[0]+preTri[1]+preTri[2]+preTri[3];
    if((preCircleFound!=circleFound)||(presum!=cursum))//state changed,set flag and update
    {
        stateChanged=true;
        preCircleFound=circleFound;
        for(int i=0;i<4;i++)
            preTri[i]=tri[i];
    }
    if(circleFound)
    {
        if(stateChanged)
        {
            for(int i=0;i<10;i++)
                write(*serialPort,boost::asio::buffer("e"),errc);
        }
        cout<<"find circle"<<endl;
    }
    else if((tri[0]+tri[1]+tri[2]+tri[3])!=0)
    {
        if(stateChanged)
        {
            for(int i=0;i<10;i++)
                write(*serialPort,boost::asio::buffer("d"),errc);
        }
        cout<<"find  triangle"<<endl;
    }
    else
    {
        if(stateChanged)
        {
            for(int i=0;i<10;i++)
                write(*serialPort,boost::asio::buffer("c"),errc);
        }
        cout<<"no circles"<<endl;
    }
}

void getHero(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("got: [%s]", msg->data.c_str());
    std::stringstream ss(msg->data.c_str());
    isVisionRunning=true;
    double y,x,z,h;
//    ss >>y>>x>>tri[0]>>tri[1]>>tri[2]>>tri[3];
    ss >>y>>x>>z>>h;
    if(fabs(x)<0.000000001&&fabs(y)<0.000000001)
        heroFound=false;
    //calculate distance
    else
    {
        heroFound=true;
        if(fabs(y)>0.000000001)
            real_dis_y=y/1000;
        if(fabs(x)>0.000000001)
            real_dis_x=(x/1000);//need calibration
        if(fabs(z)>0.000000001)//may need some change
            heroYaw=z;
        if(fabs(h)>0.000000001)
            real_height=h/1000;
        if(heroState==1)
        {
            real_dis_x=real_dis_x-0.12;
        }
        else if(heroState==2)
        {
            //do nothing
        }
        else if(heroState==3)
        {
            real_dis_y=real_dis_y-0.025;
            real_dis_x=real_dis_x+0.15;
        }
    }
    cout<<"x,y,z,height are:"<<real_dis_x<<" "<<real_dis_y<<" "<<z<<" "<<real_height<<" "<<endl;
    //write message to arduino
    bool stateChange=false;
    if(prehero!=heroFound)
    {
        prehero=heroFound;
        stateChange=true;
    }
    if(heroFound&&stateChange)
    {
        for(int i=0;i<10;i++)
            write(*serialPort,boost::asio::buffer("e"),errc);
        cout<<"find hero"<<endl;
    }
    else if((!heroFound)&&stateChange)
    {
        for(int i=0;i<10;i++)
            write(*serialPort,boost::asio::buffer("c"),errc);
        cout<<"no hero found"<<endl;
    }
}

void getRed(const std_msgs::String::ConstPtr& msg)
{
    std::stringstream ss(msg->data.c_str());
    isVisionRunning=true;

    bool rlf;
    ss >>rlf;
    redLineFound=rlf;

    //write message to arduino
    bool stateChange=false;
    if(preRed!=redLineFound)
    {
        preRed=redLineFound;
        stateChange=true;
    }
    if(redLineFound&&stateChange)
    {
        for(int i=0;i<10;i++)
            write(*serialPort,boost::asio::buffer("e"),errc);
    }
    else if((!redLineFound)&&stateChange)
    {
        for(int i=0;i<10;i++)
            write(*serialPort,boost::asio::buffer("c"),errc);
    }
    cout<<"find red"<<redLineFound<<endl;
}

void getFlightStatus(const std_msgs::UInt8::ConstPtr& msg)
{
    flightStatus=msg->data;
    cout<<"currunt status:"<<flightStatus<<endl;
}
void testVelocityControl()
{
    cout<<"mission F down"<<endl;
    double vmin=0.1;
    while(true)
    {
        ros::spinOnce();
        if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+4545)>0.000001)
            return;
        yawControl(drone,8);
//        for(int i=0;i<400;i++)
//            velocityControl(drone,vmin,vmin,0);
//        for(int i=0;i<400;i++)
//            velocityControl(drone,0,0,0);
//        ros::spinOnce();

//        for(int i=0;i<400;i++)
//            velocityControl(drone,-vmin,-vmin,0);
//        for(int i=0;i<400;i++)
//            velocityControl(drone,0,0,0);
//        ros::spinOnce();
    }
    //*****************************************************************//
}

//engineering technique contest,F up
void runEngineeringContest()
{\
    double leftrightSpeed=0.15;
    //taking off and hovering for a while
    drone->takeoff();
    sleep(8);
    //move higher
    for(int i=0;i<200;i++)
	velocityControl(drone,0,0,0.15);
    ros::spinOnce();
    //move forward for a while
    for(int i=0;i<300;i++)
    {
	velocityControl(drone,0.25,leftrightSpeed*leftRight,0);
	ros::spinOnce();
    }
    //going forward while not seeing the red circle and triangles
    while(!circleFound&&(tri[0]+tri[1]+tri[2]+tri[3])==0)
    {
        if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+10000)>0.000001)
            return;

        velocityControl(drone,0.2,leftrightSpeed*leftRight,0);
        ros::spinOnce();
    }

    //mission up--autonomous landing
    missionUp();
    sleep(1);//wait until landing on the ground

    //repeat catching ball
    for(int i=0;i<0;i++)
    {
        for(int j=0;j<10;j++)
            write(*serialPort,boost::asio::buffer("a"),errc);
        sleep(1);
        for(int j=0;j<10;j++)
            write(*serialPort,boost::asio::buffer("b"),errc);
        sleep(1);
    }
    for(int j=0;j<10;j++)
        write(*serialPort,boost::asio::buffer("a"),errc);
    sleep(4);
   // ros::spinOnce();
    //taking off and hover for a while
    drone->release_sdk_permission_control();
    sleep(2);
    drone->request_sdk_permission_control();
    sleep(2);
    drone->takeoff();
    cout<<"taking off"<<endl;
    sleep(8);
    for(int i=0;i<200;i++)
	velocityControl(drone,0,0,0.15);
    ros::spinOnce();
    for(int i=0;i<300;i++)
    {
	velocityControl(drone,-0.25,leftRight*leftrightSpeed,0);
	ros::spinOnce();
    }
    //going backward until seeing some red line
    while(!redLineFound)
    {
        if(fabs(rc_data.mode-8000)>0.000001||fabs(rc_data.gear+10000)>0.000001)
            return;

        velocityControl(drone,-0.2,leftRight*leftrightSpeed,0);
        ros::spinOnce();
    }

    //continue to move backward for a while
    for(int i=0;i<100;i++)
    {
        velocityControl(drone,-0.2,0,0);
        ros::spinOnce();
    }

    //hovering and landing
    for(int i=0;i<200;i++)               //fast landing
        velocityControl(drone,0,0,-1.0);
    drone->landing();

    //release balls
    for(int j=0;j<10;j++)
        write(*serialPort,boost::asio::buffer("b"),errc);

}

void getLeftRight(const std_msgs::String::ConstPtr& msg)
{
    isVisionRunning=true;
    std::stringstream ss(msg->data.c_str());
    int leftright;
    ss >>leftright;
    leftRight=leftright;
    cout<<"left right is:"<<leftRight<<endl;
}

void gimbalControl(DJIDrone *drone,int roll, int pitch, int yaw, int duration)
{
    drone->gimbal_angle_control(roll,pitch,yaw,duration);
}
