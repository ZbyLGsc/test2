/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both visualized in the live image and shown
 * in the text console. Optionally allows selecting of a specific
 * camera in case multiple ones are present and specifying image
 * resolution as long as supported by the camera. Also includes the
 * option to send tag detections via a serial port, for example when
 * running on a Raspberry Pi that is connected to an Arduino.
 */

#define M_DRAW true
#include "AprilTags/QRCode.h"
const char* windowName= "apriltags_demo";

vector<AprilTags::TagDetection> final_detections;

double tic()
{
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec) / 1000000.);
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch,
                  double& roll)
{
  yaw= standardRad(atan2(wRo(1, 0), wRo(0, 0)));
  double c= cos(yaw);
  double s= sin(yaw);
  pitch= standardRad(atan2(-wRo(2, 0), wRo(0, 0) * c + wRo(1, 0) * s));
  roll= standardRad(
      atan2(wRo(0, 2) * s - wRo(1, 2) * c, -wRo(0, 1) * s + wRo(1, 1) * c));
}

bool calculateCenterPointFrom3Circles(cv::Point2f Point1, float radius1,
                                      cv::Point2f Point2, float radius2,
                                      cv::Point2f Point3, float radius3,
                                      cv::Point2f& centerPoint)
{
  float x1= Point1.x, x2= Point2.x, x3= Point3.x;
  float y1= Point1.y, y2= Point2.y, y3= Point3.y;
  float D= 2 * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2));
  if(abs(D) < 1e-15)
    return false;
  float C1= radius1 * radius1 - radius2 * radius2 + x2 * x2 - x1 * x1 +
            y2 * y2 - y1 * y1;
  float C2= radius2 * radius2 - radius3 * radius3 + x3 * x3 - x2 * x2 +
            y3 * y3 - y2 * y2;
  float Dx= C1 * (y3 - y2) - C2 * (y2 - y1);
  float Dy= C2 * (x2 - x1) - C1 * (x3 - x2);
  float centerPoint_x= Dx / D, centerPoint_y= Dy / D;
  if(centerPoint_x > 2.10 || centerPoint_x < 0.0 || centerPoint_y > 2.10 ||
     centerPoint_y < 0.0)
    return false;
  centerPoint= cv::Point2f(centerPoint_x, centerPoint_y);
  return true;
}

bool reduceErrorByExtraCircle(cv::Point circlePoint, float radius,
                              float& centerPoint_x, float& centerPoint_y)
{
  float x4= circlePoint.x, y4= circlePoint.y;
  float move_vector_length=
      sqrt(pow(x4 - centerPoint_x, 2) + pow(y4 - centerPoint_y, 2));
  if(abs(move_vector_length) < 1e-6)
    return false;
  float distance_to_correct= (move_vector_length - radius) / 2;
  centerPoint_x+=
      distance_to_correct / move_vector_length * (x4 - centerPoint_x);
  centerPoint_y+=
      distance_to_correct / move_vector_length * (y4 - centerPoint_y);
  return true;
}
// default constructor
QRCode::QRCode()
  :  // default settings, most can be modified through command line
     // options (see below)
  m_tagDetector(NULL)
  , m_tagCodes(AprilTags::tagCodes16h5)
  ,

  m_draw(M_DRAW)
  , m_arduino(false)
  , m_timing(false)
  ,

  m_width(640)
  , m_height(480)
  , m_tagSize(0.250)
  , m_fx(569)
  , m_fy(568)
  , m_px(311.035)
  , m_py(248.974)
  ,

  m_exposure(-1)
  , m_gain(-1)
  , m_brightness(-1)
  ,

  m_deviceId(1)
  ,

  base_position_x(1.05)
  , base_position_y(1.05)
{
}

// changing the tag family
void QRCode::setTagCodes(string s)
{
  if(s == "16h5")
  {
    m_tagCodes= AprilTags::tagCodes16h5;
  }
  else if(s == "25h7")
  {
    m_tagCodes= AprilTags::tagCodes25h7;
  }
  else if(s == "25h9")
  {
    m_tagCodes= AprilTags::tagCodes25h9;
  }
  else if(s == "36h9")
  {
    m_tagCodes= AprilTags::tagCodes36h9;
  }
  else if(s == "36h11")
  {
    m_tagCodes= AprilTags::tagCodes36h11;
  }
  else
  {
    cout << "Invalid tag family specified" << endl;
    exit(1);
  }
}

// parse command line options to change default behavior
void QRCode::parseOptions(int argc, char* argv[])
{
  int c;
  while((c= getopt(argc, argv, ":h?adtC:F:H:S:W:E:G:B:D:")) != -1)
  {
    // Each option character has to be in the string in getopt();
    // the first colon changes the error character from '?' to ':';
    // a colon after an option means that there is an extra
    // parameter to this option; 'W' is a reserved character
    switch(c)
    {
      case 'h':
      case '?':
        cout << intro;
        cout << usage;
        exit(0);
        break;
      case 'a':
        m_arduino= true;
        break;
      case 'd':
        m_draw= false;
        break;
      case 't':
        m_timing= true;
        break;
      case 'C':
        setTagCodes(optarg);
        break;
      case 'F':
        m_fx= atof(optarg);
        m_fy= m_fx;
        break;
      case 'H':
        m_height= atoi(optarg);
        m_py= m_height / 2;
        break;
      case 'S':
        m_tagSize= atof(optarg);
        break;
      case 'W':
        m_width= atoi(optarg);
        m_px= m_width / 2;
        break;
      case 'E':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Exposure option (-E) not available" << endl;
        exit(1);
#endif
        m_exposure= atoi(optarg);
        break;
      case 'G':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Gain option (-G) not available" << endl;
        exit(1);
#endif
        m_gain= atoi(optarg);
        break;
      case 'B':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Brightness option (-B) not available" << endl;
        exit(1);
#endif
        m_brightness= atoi(optarg);
        break;
      case 'D':
        m_deviceId= atoi(optarg);
        break;
      case ':':  // unknown option, from getopt
        cout << intro;
        cout << usage;
        exit(1);
        break;
    }
  }

  if(argc > optind)
  {
    for(int i= 0; i < argc - optind; i++)
    {
      m_imgNames.push_back(argv[optind + i]);
    }
  }
}

void QRCode::setup()
{
  m_tagDetector= new AprilTags::TagDetector(m_tagCodes);

  // prepare window for drawing the camera images
  if(m_draw)
  {
    cv::namedWindow(windowName, 1);
  }

  // optional: prepare serial port for communication with Arduino
  if(m_arduino)
  {
    // m_serial.open("/dev/ttyACM0");
  }
}

/*void QRCode::setupVideo() {

#ifdef EXPOSURE_CONTROL
    // manually setting camera exposure settings; OpenCV/v4l1 doesn't
    // support exposure control; so here we manually use v4l2 before
    // opening the device via OpenCV; confirmed to work with Logitech
    // C270; try exposure=20, gain=100, brightness=150

    string video_str = "/dev/video";
    video_str[10] = '0' + m_deviceId;
    int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

    if (m_exposure >= 0) {
        // not sure why, but v4l2_set_control() does not work for
        // V4L2_CID_EXPOSURE_AUTO...
        struct v4l2_control c;
        c.id = V4L2_CID_EXPOSURE_AUTO;
        c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
        if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
            cout << "Failed to set... " << strerror(errno) << endl;
        }
        cout << "exposure: " << m_exposure << endl;
        v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE,
m_exposure*6);
    }
    if (m_gain >= 0) {
        cout << "gain: " << m_gain << endl;
        v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
    }
    if (m_brightness >= 0) {
        cout << "brightness: " << m_brightness << endl;
        v4l2_set_control(device, V4L2_CID_BRIGHTNESS,
m_brightness*256);
    }
    v4l2_close(device);
#endif

    // find and open a USB camera (built in laptop camera, web cam
etc)
    m_cap = cv::VideoCapture("/home/jachinshen/视频/arc2.avi");
    if(!m_cap.isOpened()) {
        cerr << "ERROR: Can't find video device " << m_deviceId <<
"\n";
        exit(1);
    }
    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages
above...)" << endl;
    cout << "Actual resolution: "
        << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
        << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

}*/

void QRCode::print_detection(AprilTags::TagDetection& detection) const
{
  cout << "  Id: " << detection.id << " (Hamming: " << detection.hammingDistance
       << ")";

  // recovering the relative pose of a tag:

  // NOTE: for this to be accurate, it is necessary to use the
  // actual camera parameters here as well as the actual tag size
  // (m_fx, m_fy, m_px, m_py, m_tagSize)

  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                           translation, rotation);

  Eigen::Matrix3d F;
  F << 1, 0, 0, 0, -1, 0, 0, 0, 1;
  Eigen::Matrix3d fixed_rot= F * rotation;
  double yaw, pitch, roll;
  wRo_to_euler(fixed_rot, yaw, pitch, roll);

  cout << "  distance=" << translation.norm() << "m, x=" << translation(0)
       << ", y=" << translation(1) << ", z=" << translation(2)
       << ", yaw=" << yaw << ", pitch=" << pitch << ", roll=" << roll << endl;

  // Also note that for SLAM/multi-view application it is better to
  // use reprojection error of corner points, because the noise in
  // this relative pose is very non-Gaussian; see iSAM source code
  // for suitable factors.
}

void QRCode::getDetectionLocationAndDistance(
    vector<cv::Point2f>& detections_location,
    vector<float>& detections_distance, float detections_height,
    vector<AprilTags::TagDetection>& detections)
{
  static cv::Point2f id2location[12]= {
    cv::Point2f(0.2, 0.2),   cv::Point2f(0.2, 1.05),  cv::Point2f(0.2, 1.90),
    cv::Point2f(1.05, 0.2),  cv::Point2f(1.05, 1.90), cv::Point2f(1.90, 0.2),
    cv::Point2f(1.90, 1.05), cv::Point2f(0.0, 0.0),   cv::Point2f(0.0, 0.0),
    cv::Point2f(0.0, 0.0),   cv::Point2f(1.90, 1.90)
  };

  for(int i= 0; i < detections.size(); i++)
  {
    if(detections[i].hammingDistance == 0)
    {
      if((detections[i].id >= 0 && detections[i].id <= 6) ||
         detections[i].id == 10)
      {
        detections_location.push_back(id2location[detections[i].id]);
      }
    }

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detections[i].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px,
                                                 m_py, translation, rotation);

    Eigen::Matrix3d F;
    F << 1, 0, 0, 0, -1, 0, 0, 0, 1;
    Eigen::Matrix3d fixed_rot= F * rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    float ground_distance=
        sqrt(abs(pow(translation.norm(), 2) - pow(detections_height, 2)));
    detections_distance.push_back(ground_distance);
  }
}

bool QRCode::calculateBasePostion(vector<cv::Point2f>& detections_location,
                                  vector<float>& detections_distance)
{
  int detections_cnt= detections_location.size();
  if(detections_cnt == 0)
  {
    base_position_x= 1.05;
    base_position_y= 1.05;
    return false;
  }
  if(detections_cnt == 1)
  {
    base_position_x= detections_location[0].x;
    base_position_y= detections_location[0].y;
    return true;
  }
  if(detections_cnt == 2)
  {
    base_position_x= (detections_location[0].x + detections_location[1].x) / 2;
    base_position_y= (detections_location[0].y + detections_location[1].y) / 2;
    return true;
  }
  if(detections_cnt >= 3)
  {
    cv::Point2f centerPoint;
    vector<cv::Point2f> centerPoints;
    float radius;
    // calculate centerpoint(s)
    for(int i= 0; i < detections_location.size() - 2; i++)
    {
      for(int j= i + 1; j < detections_location.size() - 1; j++)
      {
        for(int k= j + 1; k < detections_location.size(); k++)
        {
          if(calculateCenterPointFrom3Circles(
                 detections_location[i], detections_distance[i],
                 detections_location[j], detections_distance[j],
                 detections_location[k], detections_distance[k], centerPoint))
          {
            // cout<<"x:"<<centerPoint.x<<" y:"<<centerPoint.y<<endl;
            centerPoints.push_back(centerPoint);
          }
        }
      }
    }

    if(centerPoints.size() == 0)
    {
      float x_sum= 0, y_sum= 0;
      for(int i= 0; i < detections_cnt; i++)
      {
        x_sum+= detections_location[i].x;
        y_sum+= detections_location[i].y;
      }
      centerPoint.x= x_sum / detections_cnt;
      centerPoint.y= y_sum / detections_cnt;
    }
    if(centerPoints.size() > 1)
    {
      float x_sum= 0, y_sum= 0;
      for(int i= 0; i < centerPoints.size(); i++)
      {
        x_sum+= centerPoints[i].x;
        y_sum+= centerPoints[i].y;
      }
      centerPoint.x= x_sum / centerPoints.size();
      centerPoint.y= y_sum / centerPoints.size();
    }
    // cout<<"corrected x:"<<centerPoint.x<<"
    // y:"<<centerPoint.y<<endl;
    base_position_x= centerPoint.x;
    base_position_y= centerPoint.y;
    return true;
  }
}

void QRCode::processImage(cv::Mat& image, cv::Mat& image_gray,
                          vector<AprilTags::TagDetection>& detections)
{
  // alternative way is to grab, then retrieve; allows for
  // multiple grab when processing below frame rate - v4l keeps a
  // number of frames buffered, which can lead to significant lag
  //      m_cap.grab();
  //      m_cap.retrieve(image);

  // detect April tags (requires a gray scale image)
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  double t0;
  if(m_timing)
  {
    t0= tic();
  }
  detections= m_tagDetector->extractTags(image_gray);

  final_detections= detections;

  if(m_timing)
  {
    double dt= tic() - t0;
    cout << "Extracting tags took " << dt << " seconds." << endl;
  }

  // print out each detection, base_position_x, base_position_y)
  // cout << detections.size() << " tags detected:" << endl;

  // show the current image including any detections
  if(m_draw)
  {
    for(int i= 0; i < detections.size(); i++)
    {
      // also highlight in the image
      detections[i].draw(image);
    }
    imshow(windowName, image);  // OpenCV call
  }

  // optionally send tag information to serial port (e.g. to Arduino)
  if(m_arduino)
  {
    if(detections.size() > 0)
    {
      // only the first detected tag is sent out for now
      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      detections[0].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px,
                                                   m_py, translation, rotation);
      // m_serial.print(detections[0].id);
      // m_serial.print(",");
      // m_serial.print(translation(0));
      // m_serial.print(",");
      // m_serial.print(translation(1));
      // m_serial.print(",");
      // m_serial.print(translation(2));
      // m_serial.print("\n");
    }
    else
    {
      // no tag detected: tag ID = -1
      // m_serial.print("-1,0.0,0.0,0.0\n");
    }
  }
}

// Load and process a single image
/*void QRCode::loadImages() {
    cv::Mat image;
    cv::Mat image_gray;

    for (list<string>::iterator it=m_imgNames.begin();
it!=m_imgNames.end(); it++) {
        image = cv::imread(*it); // load image with opencv
        processImage(image, image_gray);
        while (cv::waitKey(100) == -1) {}
    }
}
*/
// Video or image processing?
bool QRCode::isVideo()
{
  return m_imgNames.empty();
}

// The processing loop where images are retrieved, tags detected,
// and information about detections generated
/*void QRCode::loop(int argc, char* argv[]) {

    cv::Mat image;
    cv::Mat image_gray;
    cv::Mat draw = image.clone();

    vector< cv::Point2f > detections_location;
    vector< float > detections_distance;
    float detections_height=2.5;


    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle node;
    tf::TransformBroadcaster br;
    tf::Transform transform_test;

    transform_test.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform_test.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform_test,
ros::Time::now(), "camera_rgb_optical_frame", "marker_frame"));

    ros::Rate loop_rate(20);

    int frame = 0;
    double last_t = tic();
    while (ros::ok()) {

        // capture frame
        m_cap >> image;

        processImage(image, image_gray);

        getDetectionLocationAndDistance(detections_location,
                detections_distance,
                detections_height);

        if( calculateBasePostion(detections_location,
                    detections_distance))
        {
            cout<<"base x:"<<base_position_x<<"
y:"<<base_position_y<<endl;
        }
        else
            cout<<"can't calculate base position"<<endl;
        // print out the frame rate at which image frames are being
processed
        frame++;
        if (frame % 10 == 0) {
            double t = tic();
            cout << "  " << 10./(t-last_t) << " fps" << endl;
            last_t = t;
        }

        // recovering the relative pose of a tag:

        // NOTE: for this to be accurate, it is necessary to use the
        // actual camera parameters here as well as the actual tag
size
        // (m_fx, m_fy, m_px, m_py, m_tagSize)

        /*Eigen::Vector3d translation;
          Eigen::Matrix3d rotation;
        //detection.getRelativeTranslationRotation(m_tagSize, m_fx,
m_fy, m_px, m_py,
        //                                         translation,
rotation);

        Eigen::Matrix3d F;
        F <<
        1, 0,  0,
        0,  -1,  0,
        0,  0,  1;
        Eigen::Matrix3d fixed_rot = F*rotation;
        double yaw, pitch, roll;
        wRo_to_euler(fixed_rot, yaw, pitch, roll);

        cout << "  distance=" << translation.norm()
        << "m, x=" << translation(0)
        << ", y=" << translation(1)
        << ", z=" << translation(2)
        << ", yaw=" << yaw
        << ", pitch=" << pitch
        << ", roll=" << roll
        << endl;


        transform_test.setOrigin( tf::Vector3(translation(0),
translation(1), translation(2)) );
        transform_test.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br.sendTransform(tf::StampedTransform(transform_test,
ros::Time::now(), "camera_rgb_optical_frame", "marker_frame"));
        // exit if any key is pressed
        //if (cv::waitKey(1) >= 0) break;
        cv::waitKey(0);
    }
    ros::spinOnce();
    loop_rate.sleep();
}*/

bool QRCode::getBasePosition(cv::Mat& src, float detections_height)
{
  cv::Mat image_gray;
  vector<cv::Point2f> detections_location;
  vector<float> detections_distance;
  vector<AprilTags::TagDetection> detections;

  processImage(src, image_gray, detections);
  getDetectionLocationAndDistance(detections_location, detections_distance,
                                  detections_height, detections);
  return calculateBasePostion(detections_location, detections_distance);
}

float QRCode::getBaseX()
{
  return base_position_x - 1.05;
}

float QRCode::getBaseY()
{
  return 1.05 - base_position_y;
}

void QRCode::setVisability(bool visable)
{
  m_draw= visable;
}
