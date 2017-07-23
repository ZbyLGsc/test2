#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
using namespace std;

const string usage = "\n"
"Usage:\n"
"  apriltags_demo [OPTION...] [IMG1 [IMG2...]]\n"
"\n"
"Options:\n"
"  -h  -?          Show help options\n"
"  -a              Arduino (send tag ids over serial port)\n"
"  -d              Disable graphics\n"
"  -t              Timing of tag extraction\n"
"  -C <bbxhh>      Tag family (default 16h5)\n"
"  -D <id>         Video device ID (if multiple cameras present)\n"
"  -F <fx>         Focal length in pixels\n"
"  -W <width>      Image width (default 640, availability depends on camera)\n"
"  -H <height>     Image height (default 480, availability depends on camera)\n"
"  -S <size>       Tag size (square black frame) in meters\n"
"  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
"  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
"  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
"\n";

const string intro = "\n"
"April tags test code\n"
"(C) 2012-2014 Massachusetts Institute of Technology\n"
"Michael Kaess\n"
"\n";


#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif


//ros tf
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "../AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"


// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

// For Arduino: locally defined serial port access class
//#include "Serial.h"



// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic();

#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

inline double standardRad(double t) {
    if (t >= 0.) {
        t = fmod(t+PI, TWOPI) - PI;
    } else {
        t = fmod(t-PI, -TWOPI) + PI;
    }
    return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);

bool calculateCenterPointFrom3Circles(cv::Point2f Point1, float radius1,
        cv::Point2f Point2, float radius2,
        cv::Point2f Point3, float radius3,
        cv::Point2f& centerPoint);

bool reduceErrorByExtraCircle(cv::Point circlePoint, float radius,
        float& centerPoint_x, float& centerPoint_y);
/**
 * Normalize angle to be within the interval [-pi,pi].
 */

class QRCode {

    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes m_tagCodes;

    bool m_draw; // draw image and April tag detections?
    bool m_arduino; // send tag detections to serial port?
    bool m_timing; // print timing information for each tag extraction call

    int m_width; // image size in pixels
    int m_height;
    double m_tagSize; // April tag side length in meters of square black frame
    double m_fx; // camera focal length in pixels
    double m_fy;
    double m_px; // camera principal point
    double m_py;

    
    int m_deviceId; // camera id (in case of multiple cameras)

    list<string> m_imgNames;

    cv::VideoCapture m_cap;

    int m_exposure;
    int m_gain;
    int m_brightness;

    //Serial m_serial;

    public:

    float base_position_x;
    float base_position_y;

    vector<AprilTags::TagDetection> detections;

    // default constructor
    QRCode();
    // changing the tag family
    void setTagCodes(string s);

    // parse command line options to change default behavior
    void parseOptions(int argc, char* argv[]);

    void setup();

	void setVisability(bool v);
    //void setupVideo();

    void print_detection(AprilTags::TagDetection& detection) const ;
    void getDetectionLocationAndDistance(vector< cv::Point2f >& detections_location,
                                       vector< float >& detections_distance,
                                       float detections_height);
                                       
    bool calculateBasePostion(vector< cv::Point2f >& detections_location,
                              vector< float >& detections_distance);

    void processImage(const cv::Mat& image, cv::Mat& image_gray);
    // Load and process a single image
    //void loadImages();
    // Video or image processing?
    bool isVideo();

    // The processing loop where images are retrieved, tags detected,
    // and information about detections generated
    //void loop(int argc, char* argv[]);

    bool getBasePosition(const cv::Mat& src, float detections_height);

    float getBaseX();
    float getBaseY();

    bool getBaseDirection(float& baseDirectionCita);


}; // Demo


// here is were everything begins
/*int main(int argc, char* argv[]) {

    Demo demo;
    
    // process command line options
    demo.parseOptions(argc, argv);

    demo.setup();

    // ros::init(argc, argv, "my_tf_broadcaster");
    // ros::NodeHandle node;
    // tf::TransformBroadcaster br;
    // tf::Transform transform_test;

    // vector<AprilTags::TagDetection> final_detections;

    // transform_test.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    // transform_test.setRotation( tf::Quaternion(0, 0, 0, 1) );
    // br.sendTransform(tf::StampedTransform(transform_test, ros::Time::now(), "camera_rgb_optical_frame", "marker_frame"));

    if (demo.isVideo()) {
        cout << "Processing video" << endl;

        // setup image source, window for drawing, serial port...
        demo.setupVideo();

        // the actual processing loop where tags are detected and visualized
        demo.loop(argc,argv);

    } else {
        cout << "Processing image" << endl;

        // process single image
        demo.loadImages();

    }

    return 0;
}*/

