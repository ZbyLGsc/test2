//#include <cv.h>
//#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#define rLowH 0
#define rHighH 210
#define rLowS 90
#define rHighS 230
#define rLowV 0
#define rHighV 255
#define rgThresh 40
#define rbThresh 30

#define bLowH 0
#define bHighH 210
#define bLowS 90
#define bHighS 230
#define bLowV 0
#define bHighV 255
#define brThresh 40
#define bgThresh 30

// static int riLowH = 0;
// static int riHighH = 210;

// static int riLowS = 90;
// static int riHighS = 230;

// static int riLowV = 90;
// static int riHighV = 255;

using namespace cv;
using namespace std;
using namespace boost::asio;

//#define drawArmor(...)
#define threshold1 100
// cvMat 版本

// int iLowH = 0;
// int iHighH = 179;

// int iLowS = 57;
// int iHighS = 255;

// int iLowV = 0;
// int iHighV = 182;
static int iLowH;
static int iHighH;
static int iLowS;
static int iHighS;
static int iLowV;
static int iHighV;
static int Thresh1;  // Thresh1 equals rgThresh/brThresh
static int Thresh2;  // Thresh1 equals rbThresh/bgThresh
/************************************************************************************************/
// distance of circle center
float dis_x= 0;
float dis_y= 0;
float real_x= 0;
float real_y= 0;
int sign= 0;
// triange message
int tri[4];
bool redLineFound= false;
int leftRight= 0;
// int triRight; //right up left down,1 if found,0 if not found
// int triUp ; //right up left down,1 if found,0 if not found
// int triLeft; //right up left down,1 if found,0 if not found
// int triDown; //right up left down,1 if found,0 if not found
// publish real_x,real_y,tri[0-3],6 byte
/************************************************************************************************/

Mat drawCircle;

int color_signal= 0;  // 0 is red/else is blue
int taskFlag= 0;      // autonomous landing:0,following hero 1
void getFlag(const std_msgs::String::ConstPtr &msg);
void findRed(Mat src, bool &redLineFound);
void findBlack(Mat src, int &direction);

enum DIR
{
  RIGHT,
  UP,
  LEFT,
  DOWN
};
double angle(Point pt1, Point pt2,
             Point pt0)  // p0 angle point,return the cosine of
                         // angle
{
  double dx1= pt1.x - pt0.x;
  double dy1= pt1.y - pt0.y;
  double dx2= pt2.x - pt0.x;
  double dy2= pt2.y - pt0.y;
  //    double ratio=(dx1*dx1+dy1*dy1)/(dx2*dx2+dy2*dy2);
  //    if (ratio<0.8 ||
  //    1.25<ratio)//根据边长平方的比过小或过大提前淘汰这个四边形，如果淘汰过多，调整此比例数字
  //    return 1.0;//根据边长平方的比过小或过大提前淘汰这个四边形
  double cosine=
      (dx1 * dx2 + dy1 * dy2) /
      sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
  return acos(cosine);
}

void findHero(Mat src, float &x, float &y, double &zAngle,
              double &radius)
{
  if(src.channels() != 3)
    return;
  //        cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0
  //        - 255)
  //        cvCreateTrackbar("HighH", "Control", &iHighH, 255);

  //        cvCreateTrackbar("LowS", "Control", &iLowS, 255);
  //        //Saturation (0
  //        - 255)
  //        cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  //        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value
  //        (0 -
  //        255)
  //        cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  //        cvCreateTrackbar("br", "Control", &Thresh1, 255); //Value
  //        (0 -
  //        255)
  //        cvCreateTrackbar("bg", "Control", &Thresh2, 255);
  // to gray scale
  Mat gray;
  // cv::cvtColor(src,gray,COLOR_RGB2GRAY);
  Mat hsvbin;
  // cv::threshold(gray,bin,130,255,THRESH_BINARY);
  GaussianBlur(src, src, Size(5, 5), 0, 0);
  cvtColor(src, gray, CV_BGR2HSV);
  vector<Mat> hsvSplit;
  split(gray, hsvSplit);
  equalizeHist(hsvSplit[2], hsvSplit[2]);
  merge(hsvSplit, gray);

  inRange(gray, Scalar(iLowH, iLowS, iLowV),
          Scalar(iHighH, iHighS, iHighV),
          hsvbin);  // Threshold the image by hsv
  // threshold by r-g,r-b
  vector<Mat> bgrSplit;
  split(src, bgrSplit);
  // region that r>b and region where |r-g|>t
  Mat rlb, rmbabs, rb;
  cv::compare(bgrSplit.at(2), bgrSplit.at(0), rlb, CMP_GT);
  cv::absdiff(bgrSplit.at(2), bgrSplit.at(0), rmbabs);
  cv::threshold(rmbabs, rmbabs, rbThresh, 255, THRESH_BINARY);
  cv::bitwise_and(rlb, rmbabs, rb);
  // region where r>g and //region where |r-b|>t
  Mat rlg, rmgabs, rg;
  cv::compare(bgrSplit.at(2), bgrSplit.at(1), rlg, CMP_GT);
  cv::absdiff(bgrSplit.at(2), bgrSplit.at(1), rmgabs);
  cv::threshold(rmgabs, rmgabs, rgThresh, 255, THRESH_BINARY);
  cv::bitwise_and(rlg, rmgabs, rg);
  // all region &
  Mat target;
  cv::bitwise_and(rb, rg, target);
  cv::bitwise_and(target, hsvbin, target);

  //        cv::imshow("hero bin",target);
  // find contour
  vector<vector<Point> > contours;
  Mat temp;
  target.copyTo(temp);
  cv::findContours(temp, contours, CV_RETR_LIST,
                   CV_CHAIN_APPROX_SIMPLE);
  Mat drawContours;
  src.copyTo(drawContours);
  for(int i= 0; i < contours.size(); i++)
  {
    int area= contourArea(contours.at(i));
    if(area > 900)
    {
      cv::drawContours(drawContours, contours, i, Scalar(255, 0, 0),
                       2);
    }
  }
  //    imshow("contours",drawContours);
  // find circle
  Mat drawCircle;
  src.copyTo(drawCircle);
  Point pl(0, 240);
  Point pr(640, 240);
  Point pu(320, 0);
  Point pd(320, 480);
  line(drawCircle, pl, pr, Scalar(0, 255, 0), 1);
  line(drawCircle, pu, pd, Scalar(0, 255, 0), 1);
  vector<vector<Point> > circles;
  vector<int> radiuses;
  for(int i= 0; i < contours.size(); i++)
  {
    int area= contourArea(contours.at(i));
    if(area > 900)
    {
      Point2f center(320, 240);
      float radius= 0;
      cv::minEnclosingCircle(contours[i], center, radius);
      int area= contourArea(contours[i], false);
      float circleArea= 3.141592 * radius * radius;
      float r= area / circleArea;
      if(r > 0.8)
      {
        circles.push_back(contours.at(i));
        radiuses.push_back(radius);
      }
    }
  }
  // find large circle
  if(circles.size() == 0)
    return;
  int max= 0;
  int maxId= -1;
  for(int i= 0; i < radiuses.size(); i++)
  {
    if(radiuses.at(i) > max)
    {
      max= radiuses.at(i);
      maxId= i;
    }
  }
  cv::drawContours(drawCircle, circles, maxId, Scalar(0, 255, 255),
                   2);
  //                circle(drawCircle,center,3,Scalar(125,0,200),-1,8,0);
  // extract big circle
  Rect rect= cv::boundingRect(circles.at(maxId));
  cv::rectangle(drawCircle, rect, Scalar(0, 0, 255), 2);
  x= rect.x + rect.width / 2;
  x-= 320;
  y= rect.y + rect.height / 2;
  y= (240 - y);
  radius= (rect.width / 2 + rect.height / 2) / 2;
  Mat icon;
  target(rect).copyTo(icon);
  //        cv::imshow("hero circles",drawCircle);
  cv::resize(icon, icon, Size(480, 480));
  // find center of while
  int gravx= 0, gravy= 0, num= 0;
  for(int i= 0; i < icon.rows; i++)
  {
    for(int j= 0; j < icon.cols; j++)
    {
      int dist= sqrt(pow(i - 240, 2) + pow(j - 240, 2));
      //                        cout<<"("<<i<<","<<j<<"):"<<int(icon.at<uchar>(i,j))<<"
      //                        "<<dist<<endl;
      if(int(icon.at<uchar>(i, j)) < 100 && dist < 240)
      {
        gravy+= i;
        gravx+= j;
        num++;
        //                                cout<<i<<","<<j<<endl;
      }
    }
  }
  gravx= gravx / num;
  gravy= gravy / num;
  cv::circle(icon, Point(gravx, gravy), 10, Scalar(255, 0, 0), 2);
  //        cv::imshow("icon",icon);
  // calculate angle
  int corx= gravx - 240;
  int cory= 240 - gravy;

  double angle= 57.3 * atan2(cory, corx);
  zAngle= angle;
  std::cout << "angle is :" << angle << endl;
  //        std::cout<<"gravitational center is :"<<gravx<<"
  //        "<<gravy<<"
  //        "<<num<<endl;
}

void findCircle(Mat src, Point2f &circle_center, double &rad)
{
  if(src.channels() != 3)
    return;
  // to gray scale
  Mat gray;
  // cv::cvtColor(src,gray,COLOR_RGB2GRAY);
  Mat hsvbin;
  // cv::threshold(gray,bin,130,255,THRESH_BINARY);
  GaussianBlur(src, src, Size(5, 5), 0, 0);
  cvtColor(src, gray, CV_BGR2HSV);
  vector<Mat> hsvSplit;
  split(gray, hsvSplit);
  equalizeHist(hsvSplit[2], hsvSplit[2]);
  merge(hsvSplit, gray);

  // if(color_signal==0)
  //{
  ////创建调参模块
  // iLowH = rLowH;
  // iHighH = rHighH;

  // iLowS = rLowS;
  // iHighS = rHighS;

  // iLowV = rLowV;
  // iHighV = rHighV;

  // Thresh1=rgThresh;
  // Thresh2=rbThresh;
  //}
  // else
  //{
  // iLowH = bLowH;
  // iHighH = bHighH;

  // iLowS = bLowS;
  // iHighS = bHighS;

  // iLowV = bLowV;
  // iHighV = bHighV;

  // Thresh1=brThresh;
  // Thresh2=bgThresh;
  //}

  // static int rgThresh=40;
  // static int rbThresh=30;

  // static int brThresh=40;
  // static int bgThresh=30;
  // namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window
  // called
  // "Control"
  // imshow("Control",src);
  //// Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 255);  // Hue (0 - 255)
  cvCreateTrackbar("HighH", "Control", &iHighH, 255);

  cvCreateTrackbar("LowS", "Control", &iLowS,
                   255);  // Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV,
                   255);  // Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  // cvCreateTrackbar("rg", "Control", &rgThresh, 255); //Value (0 -
  // 255)
  // cvCreateTrackbar("rb", "Control", &rbThresh, 255);

  cvCreateTrackbar("br", "Control", &Thresh1,
                   255);  // Value (0 - 255)
  cvCreateTrackbar("bg", "Control", &Thresh2, 255);
  //  inRange(gray, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH,
  //  iHighS,
  //  iHighV),hsvbin);
  //  //Threshold the image by hsv
  // Mat target;
  inRange(gray, Scalar(iLowH, iLowS, iLowV),
          Scalar(iHighH, iHighS, iHighV),
          hsvbin);  // Threshold the image by hsv

  // threshold by r-g,r-b
  //    vector<Mat > bgrSplit;
  //    split(src,bgrSplit);
  //    //region that r>b and region where |r-g|>t
  //    Mat rlb,rmbabs,rb;
  //    cv::compare(bgrSplit.at(2),bgrSplit.at(0),rlb,CMP_GT);
  //    cv::absdiff(bgrSplit.at(2),bgrSplit.at(0),rmbabs);
  //    cv::threshold(rmbabs,rmbabs,rbThresh,255,THRESH_BINARY);
  //    cv::bitwise_and(rlb,rmbabs,rb);
  //    //region where r>g and //region where |r-b|>t
  //    Mat rlg,rmgabs,rg;
  //    cv::compare(bgrSplit.at(2),bgrSplit.at(1),rlg,CMP_GT);
  //    cv::absdiff(bgrSplit.at(2),bgrSplit.at(1),rmgabs);
  //    cv::threshold(rmgabs,rmgabs,rgThresh,255,THRESH_BINARY);
  //    cv::bitwise_and(rlg,rmgabs,rg);
  //    //all region &
  //    Mat target;
  //    cv::bitwise_and(rb,rg,target);
  //    cv::bitwise_and(target,hsvbin,target);

  // if blue color is used, threshold by b-r,b-g
  Mat target;
  if(color_signal == 0)
  {
    vector<Mat> bgrSplit;
    split(src, bgrSplit);
    // region that r>b and region where |b-r|>t
    Mat blr, bmrabs, br;
    cv::compare(bgrSplit.at(2), bgrSplit.at(1), blr, CMP_GT);
    cv::absdiff(bgrSplit.at(2), bgrSplit.at(1), bmrabs);
    cv::threshold(bmrabs, bmrabs, Thresh1, 255,
                  THRESH_BINARY);  // Thresh1 equals rgThresh/brThresh
    cv::bitwise_and(blr, bmrabs, br);
    cout << "test1" << endl;
    // region where r>g and //region where |b-g|>t
    Mat blg, bmgabs, bg;
    cv::compare(bgrSplit.at(2), bgrSplit.at(1), blg, CMP_GT);
    cv::absdiff(bgrSplit.at(2), bgrSplit.at(1), bmgabs);
    cv::threshold(bmgabs, bmgabs, Thresh2, 255,
                  THRESH_BINARY);  // Thresh1 equals rbThresh/bgThresh
    cv::bitwise_and(blg, bmgabs, bg);
    cout << "test2" << endl;
    // all region &
    cv::bitwise_and(br, bg, target);
    cv::bitwise_and(target, hsvbin, target);
  }
  else
  {
    vector<Mat> bgrSplit;
    split(src, bgrSplit);
    // region that r>b and region where |b-r|>t
    Mat blr, bmrabs, br;
    cv::compare(bgrSplit.at(0), bgrSplit.at(2), blr, CMP_GT);
    cv::absdiff(bgrSplit.at(0), bgrSplit.at(2), bmrabs);
    cv::threshold(bmrabs, bmrabs, Thresh1, 255,
                  THRESH_BINARY);  // Thresh1 equals rgThresh/brThresh
    cv::bitwise_and(blr, bmrabs, br);
    cout << "test1" << endl;
    // region where r>g and //region where |b-g|>t
    Mat blg, bmgabs, bg;
    cv::compare(bgrSplit.at(0), bgrSplit.at(1), blg, CMP_GT);
    cv::absdiff(bgrSplit.at(0), bgrSplit.at(1), bmgabs);
    cv::threshold(bmgabs, bmgabs, Thresh2, 255,
                  THRESH_BINARY);  // Thresh1 equals rbThresh/bgThresh
    cv::bitwise_and(blg, bmgabs, bg);
    cout << "test2" << endl;
    // all region &
    cv::bitwise_and(br, bg, target);
    cv::bitwise_and(target, hsvbin, target);
  }
  //    cv::imshow("bin",target);
  // find contour
  vector<vector<Point> > contours;
  Mat temp;
  target.copyTo(temp);
  cv::findContours(temp, contours, CV_RETR_LIST,
                   CV_CHAIN_APPROX_SIMPLE);
  Mat drawContours;
  src.copyTo(drawContours);
  for(int i= 0; i < contours.size(); i++)
  {
    int area= contourArea(contours.at(i));
    if(area > 900)
    {
      cv::drawContours(drawContours, contours, i, Scalar(255, 0, 0),
                       2);
    }
  }
  // imshow("contours",drawContours);
  // find circle
  // Mat drawCircle;
  src.copyTo(drawCircle);
  for(int i= 0; i < contours.size(); i++)
  {
    int area= contourArea(contours.at(i));
    if(area > 900)
    {
      Point2f center(320, 240);
      float radius= 0;
      cv::minEnclosingCircle(contours[i], center, radius);
      int area= contourArea(contours[i], false);
      float circleArea= 3.141592 * radius * radius;
      float r= area / circleArea;
      if(r > 0.8)
      {
        cv::drawContours(drawCircle, contours, i, Scalar(0, 255, 255),
                         2);
        circle(drawCircle, center, 3, Scalar(125, 0, 200), -1, 8, 0);
        circle_center= center;
        rad= radius;
      }
    }
  }
  // draw line to center
  Point pl(0, 240);
  Point pr(640, 240);
  Point pu(320, 0);
  Point pd(320, 480);
  Point cen(320, 240);
  Point ccen(320, 240);
  ccen= circle_center;
  line(drawCircle, pl, pr, Scalar(0, 255, 0), 1);
  line(drawCircle, pu, pd, Scalar(0, 255, 0), 1);
  if(circle_center.x != 0 && circle_center.y != 0)
    line(drawCircle, ccen, cen, Scalar(0, 0, 255), 2);

  ////////////////////////////////////////////////////////////////////////////////////////////
  // find triangle
  vector<vector<Point> > triangles;
  //    triRight=0;
  //    triUp=0;
  //    triLeft=0;
  //    triDown=0;
  //    DIR dir;
  for(int k= 0; k < 4; k++)
    tri[k]= 0;
  for(int i= 0; i < contours.size(); i++)
  {
    int area= contourArea(contours.at(i));

    if(area < 80)
      continue;
    cout << "area is:" << area << endl;
    //            cout<<"area is:"<<area<<endl;
    // polygon fitting,find triangles
    vector<Point> approx;
    cv::approxPolyDP(Mat(contours[i]), approx,
                     arcLength(Mat(contours[i]), true) * 0.045, true);
    //            cout<<"edge number is:"<<approx.size()<<endl;
    if(approx.size() != 3)
      continue;
    if(!isContourConvex(Mat(approx)))
      continue;

    // fitting 45 45 90 degree of triangle
    double angle1=
        angle(approx.at(0), approx.at(1), approx.at(2)) * 57.3;
    double angle2=
        angle(approx.at(2), approx.at(0), approx.at(1)) * 57.3;
    double angle3=
        angle(approx.at(2), approx.at(1), approx.at(0)) * 57.3;
    bool rightShape= false;
    double angleThreshold= 10.0;
    int verPointId;
    if(fabs(angle1 - 45) < angleThreshold &&
       fabs(angle2 - 45) < angleThreshold &&
       fabs(angle3 - 90) < angleThreshold)
    {
      rightShape= true;
      verPointId= 0;
    }
    else if(fabs(angle1 - 45) < angleThreshold &&
            fabs(angle2 - 90) < angleThreshold &&
            fabs(angle3 - 45) < angleThreshold)
    {
      rightShape= true;
      verPointId= 1;
    }
    else if(fabs(angle1 - 90) < angleThreshold &&
            fabs(angle2 - 45) < angleThreshold &&
            fabs(angle3 - 45) < angleThreshold)
    {
      rightShape= true;
      verPointId= 2;
    }
    if(!rightShape)
      continue;

    // cout<<"2 point's x = "<<approx.at(2).x<<" y =
    // "<<approx.at(2).y<<endl;
    // cout<<"1 point's x = "<<approx.at(1).x<<" y =
    // "<<approx.at(1).y<<endl;
    // cout<<"0 point's x = "<<approx.at(0).x<<" y =
    // "<<approx.at(0).y<<endl;

    cv::drawContours(drawCircle, contours, i, Scalar(0, 0, 255), 2);
    // judge the direction of the triangle
    DIR dir;
    Scalar dirColor;  // color indicating
                      // direction,red,yellow,blue,green
                      // for right up
                      // left down
    if(approx.at(verPointId).x > approx.at((verPointId + 1) % 3).x &&
       approx.at(verPointId).x >
           approx.at((verPointId + 2) % 3).x)  // left
    {
      tri[2]= 1;
      dir= LEFT;
      dirColor= Scalar(255, 0, 0);
    }
    else if(approx.at(verPointId).x <
                approx.at((verPointId + 1) % 3).x &&
            approx.at(verPointId).x <
                approx.at((verPointId + 2) % 3).x)  // right
    {
      tri[0]= 1;
      dirColor= Scalar(0, 0, 255);
      dir= RIGHT;
    }
    else if(approx.at(verPointId).y <
                approx.at((verPointId + 1) % 3).y &&
            approx.at(verPointId).y <
                approx.at((verPointId + 2) % 3).y)  // DOWN
    {
      tri[3]= 1;
      dir= DOWN;
      dirColor= Scalar(0, 255, 0);
    }
    else if(approx.at(verPointId).y >
                approx.at((verPointId + 1) % 3).y &&
            approx.at(verPointId).y >
                approx.at((verPointId + 2) % 3).y)  // UP
    {
      tri[1]= 1;
      dir= UP;
      dirColor= Scalar(0, 255, 255);
    }
    //            cv::circle(drawCircle,approx.at(verPointId),3,dirColor,4);

    // direction vector
    Point dirVec;
    Point end(approx.at(verPointId).x, approx.at(verPointId).y);
    Point start((approx.at((verPointId + 1) % 3).x +
                 approx.at((verPointId + 2) % 3).x) /
                    2,
                (approx.at((verPointId + 1) % 3).y +
                 approx.at((verPointId + 2) % 3).y) /
                    2);
    dirVec.x= (end.x - start.x);
    dirVec.y= -(end.y - start.y);
    cv::circle(drawCircle, end, 3, dirColor, 4);
    cv::circle(drawCircle, start, 3, dirColor, 2);
    cv::line(drawCircle, end, start, dirColor, 2);
    cout << "angle are:" << angle1 << " " << angle2 << " " << angle3
         << " " << endl
         << "area is:" << area << endl
         << "direction is:" << dirVec.x << " " << dirVec.y << " "
         << int(dir) << endl
         << endl;
    triangles.push_back(contours.at(i));
  }
  //        cv::drawContours(drawCircle,triangles,-1,Scalar(0,0,255),2);
  //    cv::imshow("circles",drawCircle);
  cout << "loop" << endl;
}

int undistort_pic(Mat &src)
{
}

int main(int argc, char *argv[])
{
  //读取视频文件
  cv::VideoCapture cap(0);
  if(!cap.isOpened())
    return -1;
  // double rate=capture.get(CV_CAP_PROP_FPS);
  Mat frame, tmp, dst, gray, hsv, result, canny;

  // ros publisher
  ros::init(argc, argv, "vision");
  ros::NodeHandle n;
  ros::Publisher landing_pub=
      n.advertise<std_msgs::String>("tpp/landing", 10);
  ros::Publisher hero_pub=
      n.advertise<std_msgs::String>("tpp/hero", 10);
  ros::Publisher red_pub=
      n.advertise<std_msgs::String>("tpp/red", 10);
  ros::Publisher leftright_pub=
      n.advertise<std_msgs::String>("tpp/leftright", 10);

  ros::Subscriber color_sub= n.subscribe("tpp/flag", 1, getFlag);
  ros::Rate loop_rate(100);

  // information about camera
  // double camera_matrix_data[3][3] =
  //{
  // {932.843,0,320.735},
  // {0,933.185,245.333},
  // {0,0,1}
  //};
  double camera_matrix_data[3][3]= { { 508.013, 0, 322.632 },
                                     { 0, 507.49, 231.39 },
                                     { 0, 0, 1 } };
  // double distortcof_data[5] =
  // {-0.395513,0.00157651,-0.00353122,0.000820458,1.61204};
  double distortcof_data[5]= { 0.106666, -0.298727, 0.00059,
                               -0.0022715, 0.223131 };
  double focus_distance=
      (camera_matrix_data[0][0] + camera_matrix_data[1][1]) / 2;
  double pic_distance= 0.0;
  double real_distance= 0.0;
  double height= 0.0;
  double zAngle= 0.0;
  Mat cameramatrix(3, 3, CV_64F, camera_matrix_data);
  Mat distortcof(5, 1, CV_64F, distortcof_data);

  // int delay=1000/rate;
  //创建调参模块
  // namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window
  // called
  // "Control"
  //实验出的最佳参数：H(0,15),S(130,255),V(90,255)
  //  int iLowH = 0;
  //  int iHighH = 150;

  //  int iLowS = 0;
  //  int iHighS = 255;

  //  int iLowV = 0;
  //  int iHighV = 255;

  // Create trackbars in "Control" window
  //  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 -
  //  179)
  //  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  //  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation
  //  (0 - 255)
  //  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  //  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 -
  //  255)
  //  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  VideoWriter write;
  string outFile= "/home/ubuntu/dji_vision_mzm/record0.avi";
  int w= 640;
  int h= 480;
  Size S(w, h);
  // double r = cap.get(CV_CAP_PROP_FPS);

  write.open(outFile, CV_FOURCC('M', 'J', 'P', 'G'), 30, S, true);

  if(color_signal == 0)
  {
    //创建调参模块
    iLowH= rLowH;
    iHighH= rHighH;

    iLowS= rLowS;
    iHighS= rHighS;

    iLowV= rLowV;
    iHighV= rHighV;

    Thresh1= rgThresh;
    Thresh2= rbThresh;
  }
  else
  {
    iLowH= bLowH;
    iHighH= bHighH;

    iLowS= bLowS;
    iHighS= bHighS;

    iLowV= bLowV;
    iHighV= bHighV;

    Thresh1= brThresh;
    Thresh2= bgThresh;
  }

  while(ros::ok())
  {
    cap >> frame;
    //压缩图片
    tmp= frame;
    dst= tmp;
    // pyrDown( tmp, dst, Size( tmp.cols/2, tmp.rows/2 ) );
    cv::resize(tmp, dst, Size(640, 480));
    // write.write(frame);

    //转换成HSV格式
    // GaussianBlur(dst,dst,Size(3,3),0,0);
    // cvtColor(dst, hsv, CV_BGR2HSV );
    // vector<Mat> hsvSplit;
    // split(hsv, hsvSplit);
    // equalizeHist(hsvSplit[2],hsvSplit[2]);
    // merge(hsvSplit,hsv);
    // inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH,
    // iHighS,
    // iHighV),result); //Threshold the image

    //去除噪点，连接连通域
    // Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));
    // morphologyEx(result, result, MORPH_OPEN, element);
    // morphologyEx(result, result, MORPH_CLOSE, element);

    //膨胀
    // dilate(result,result,Mat(3,3,CV_8U),Point(-1,-1),1);

    // undistort(tmp,result,cameramatrix,distortcof);
    // cv::resize(result,result,Size(640,480));
    // imshow("1",result);
    if(taskFlag == 0)  // autonomous landing
    {
      // calculating distance
      Point2f land_center(0, 0);
      double radius= 0.0;
      findCircle(dst, land_center, radius);
      dis_x= land_center.x - 320;  // distance to image center
      dis_y= (-1) * (land_center.y - 240);
      pic_distance= sqrt(dis_x * dis_x + dis_y * dis_y);
      cout << "dis x = " << dis_x << " y = " << dis_y
           << " radius = " << radius << endl;
      cout << "focus_distance = " << focus_distance << endl;
      height=
          (250 / radius) * focus_distance;  // from camera to board
      real_distance=
          (250 / radius) *
          pic_distance;  // from camera center to circle center
      real_x= (dis_x / pic_distance) * real_distance;
      real_y= (dis_y / pic_distance) * real_distance;
      if((dis_x == -320) && (dis_y == 240) && (radius == 0))
      {
        real_x= 0;
        real_y= 0;
        real_distance= 0;
        height= 0;
      }
      cout << "real x = " << real_x << "mm real y = " << real_y
           << "mm" << endl;
      cout << "real distance = " << real_distance << "mm" << endl;
      for(int k= 0; k < 4; k++)
        cout << tri[k] << " ";
      cout << endl;
      cout << "height = " << height << endl;
      std_msgs::String msg;
      std::stringstream ss;
      // ss <<real_x<<" "<<real_y<<" "<<triRight<<" "<<triUp<"
      // "<<triLeft<<"
      // "<<triDown<<" ";
      ss << tri[0] << " " << tri[1] << " " << tri[2] << " " << tri[3]
         << " " << real_x << " " << real_y << " " << height;
      msg.data= ss.str();

      ROS_INFO("%s", msg.data.c_str());
      landing_pub.publish(msg);

      // find red
      findRed(dst, redLineFound);
      std_msgs::String msgRed;
      std::stringstream ssRed;
      ssRed << redLineFound;
      msgRed.data= ssRed.str();
      red_pub.publish(msgRed);

      // find black center
      findBlack(dst, leftRight);
      std_msgs::String msglr;
      std::stringstream sslr;
      sslr << leftRight;
      msglr.data= sslr.str();
      leftright_pub.publish(msglr);
    }
    else if(taskFlag == 1)  // find hero
    {
      double radius= 0.0;
      findHero(dst, dis_x, dis_y, zAngle, radius);
      height= (75 / radius) * focus_distance;
      pic_distance= sqrt(dis_x * dis_x + dis_y * dis_y);
      real_distance=
          (75 / radius) *
          pic_distance;  // from camera center to circle center
      real_x= (dis_x / pic_distance) * real_distance;
      real_y= (dis_y / pic_distance) * real_distance;
      if((dis_x == -320) && (dis_y == 240) && (radius == 0))
      {
        real_x= 0;
        real_y= 0;
        real_distance= 0;
        height= 0;
        zAngle= 0;
      }

      std_msgs::String msg;
      std::stringstream ss;
      ss << real_x << " " << real_y << " " << zAngle << " " << height;
      //      cout<<"hero:"<<real_x<<" "<<real_y<<" "<<zAngle<<"
      //      "<<height<<endl;
      msg.data= ss.str();
      ROS_INFO("%s", msg.data.c_str());
      hero_pub.publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
    // write.write(drawCircle);

    //显示压缩后原图
    // imshow( "yasuo", dst);
    //显示HSV提取后图像
    // imshow("hsv",canny);
    waitKey(1);
  }
  cap.release();
  // write.release();
}

void getFlag(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("got: [%s]", msg->data.c_str());
  std::stringstream ss(msg->data.c_str());

  char flag;
  //    ss >>y>>x>>tri[0]>>tri[1]>>tri[2]>>tri[3];
  ss >> flag;

  if(flag == 'r')
  {
    color_signal= 0;
    iLowH= rLowH;
    iHighH= rHighH;

    iLowS= rLowS;
    iHighS= rHighS;

    iLowV= rLowV;
    iHighV= rHighV;

    Thresh1= rgThresh;
    Thresh2= rbThresh;
    cout << "current color change to red:" << color_signal << endl;
    //        cv::waitKey();
  }
  else if(flag == 'b')
  {
    color_signal= 1;
    iLowH= bLowH;
    iHighH= bHighH;

    iLowS= bLowS;
    iHighS= bHighS;

    iLowV= bLowV;
    iHighV= bHighV;

    Thresh1= brThresh;
    Thresh2= bgThresh;
    cout << "current color change to blue:" << color_signal << endl;
    //        cv::waitKey();
  }
  else if(flag == 'l')
  {
    taskFlag= 0;
    cout << "current task is autonomous landing:" << taskFlag << endl;
  }
  else if(flag == 'h')
  {
    taskFlag= 1;
    cout << "current task is following hero:" << taskFlag << endl;
  }
  else
  {
    cout << "color error" << endl;
  }
}

void findRed(Mat src, bool &redLineFound)
{
  if(src.channels() != 3)
    return;
  // image pre-processing
  Mat gray;
  // cv::cvtColor(src,gray,COLOR_RGB2GRAY);
  Mat hsvbin;
  // cv::threshold(gray,bin,130,255,THRESH_BINARY);
  GaussianBlur(src, src, Size(5, 5), 0, 0);
  cvtColor(src, gray, CV_BGR2HSV);
  vector<Mat> hsvSplit;
  split(gray, hsvSplit);
  equalizeHist(hsvSplit[2], hsvSplit[2]);
  merge(hsvSplit, gray);

  //创建调参模块
  //    static int iLowH = 0;
  //    static int iHighH = 210;

  //    static int iLowS = 90;
  //    static int iHighS = 230;

  //    static int iLowV = 0;
  //    static int iHighV = 255;

  //    static int rgThresh=40;
  //    static int rbThresh=30;
  //     namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window
  //     called
  //     "Control"
  //     imshow("Control",src);
  //    // Create trackbars in "Control" window
  //     cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 -
  //     255)
  //     cvCreateTrackbar("HighH", "Control", &iHighH, 255);

  //     cvCreateTrackbar("LowS", "Control", &iLowS, 255);
  //     //Saturation (0 -
  //     255)
  //     cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  //     cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0
  //     - 255)
  //     cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  //     cvCreateTrackbar("rg", "Control", &rgThresh, 255); //Value (0
  //     - 255)
  //     cvCreateTrackbar("rb", "Control", &rbThresh, 255);
  inRange(gray, Scalar(iLowH, iLowS, iLowV),
          Scalar(iHighH, iHighS, iHighV),
          hsvbin);  // Threshold the image by hsv
  // threshold by r-g,r-b
  vector<Mat> bgrSplit;
  split(src, bgrSplit);
  // region that r>b and region where |r-g|>t
  Mat rlb, rmbabs, rb;
  cv::compare(bgrSplit.at(2), bgrSplit.at(0), rlb, CMP_GT);
  cv::absdiff(bgrSplit.at(2), bgrSplit.at(0), rmbabs);
  cv::threshold(rmbabs, rmbabs, rbThresh, 255, THRESH_BINARY);
  cv::bitwise_and(rlb, rmbabs, rb);
  // region where r>g and //region where |r-b|>t
  Mat rlg, rmgabs, rg;
  cv::compare(bgrSplit.at(2), bgrSplit.at(1), rlg, CMP_GT);
  cv::absdiff(bgrSplit.at(2), bgrSplit.at(1), rmgabs);
  cv::threshold(rmgabs, rmgabs, rgThresh, 255, THRESH_BINARY);
  cv::bitwise_and(rlg, rmgabs, rg);
  // all region &
  Mat target;
  cv::bitwise_and(rb, rg, target);
  cv::bitwise_and(target, hsvbin, target);
  //        cv::imshow("bin",target);

  // count number of red point
  int num= 0;
  for(int i= 0; i < target.rows; i++)
  {
    for(int j= 0; j < target.cols; j++)
    {
      if(int(target.at<uchar>(i, j)) > 100)
        num++;
    }
  }
  if(num > 10000)
    redLineFound= true;
  else
    redLineFound= false;
  cout << "red points number  is:" << num
       << "red line found is:" << redLineFound << endl;
}

void findBlack(Mat src, int &direction)
{
  if(src.channels() != 3)
    return;
  // image pre-processing
  Mat gray;
  // cv::cvtColor(src,gray,COLOR_RGB2GRAY);
  Mat hsvbin;
  // cv::threshold(gray,bin,130,255,THRESH_BINARY);
  GaussianBlur(src, src, Size(5, 5), 0, 0);
  cvtColor(src, gray, CV_BGR2HSV);
  vector<Mat> hsvSplit;
  split(gray, hsvSplit);
  equalizeHist(hsvSplit[2], hsvSplit[2]);
  merge(hsvSplit, gray);

  //    //åå»ºè°åæš¡å
  //    static int iLowH = 0;
  //    static int iHighH = 210;

  //    static int iLowS = 90;
  //    static int iHighS = 230;

  //    static int iLowV = 0;
  //    static int iHighV = 255;

  //    static int rgThresh=40;
  //    static int rbThresh=30;
  //åå»ºè°åæš¡å
  static int iLowH_b= 0;
  static int iHighH_b= 255;

  static int iLowS_b= 0;
  static int iHighS_b= 52;

  static int iLowV_b= 0;
  static int iHighV_b= 22;

  //        static int rgThresh=40;
  //        static int rbThresh=30;
  //         namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a
  //         window
  //         called
  //         "Control"
  //         imshow("Control",src);
  //        // Create trackbars in "Control" window
  //         cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue
  //         (0 - 255)
  //         cvCreateTrackbar("HighH", "Control", &iHighH, 255);

  //         cvCreateTrackbar("LowS", "Control", &iLowS, 255);
  //         //Saturation (0
  //         - 255)
  //         cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  //         cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value
  //         (0 -
  //         255)
  //         cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  //     cvCreateTrackbar("rg", "Control", &rgThresh, 255); //Value (0
  //     - 255)
  //     cvCreateTrackbar("rb", "Control", &rbThresh, 255);
  inRange(gray, Scalar(iLowH_b, iLowS_b, iLowV_b),
          Scalar(iHighH_b, iHighS_b, iHighV_b),
          hsvbin);  // Threshold the image by hsv
  // threshold by r-g,r-b
  //        vector<Mat > bgrSplit;
  //        split(src,bgrSplit);
  //        //region that r>b and region where |r-g|>t
  //        Mat rlb,rmbabs,rb;
  //        cv::compare(bgrSplit.at(2),bgrSplit.at(0),rlb,CMP_GT);
  //        cv::absdiff(bgrSplit.at(2),bgrSplit.at(0),rmbabs);
  //        cv::threshold(rmbabs,rmbabs,rbThresh,255,THRESH_BINARY);
  //        cv::bitwise_and(rlb,rmbabs,rb);
  //        //region where r>g and //region where |r-b|>t
  //        Mat rlg,rmgabs,rg;
  //        cv::compare(bgrSplit.at(2),bgrSplit.at(1),rlg,CMP_GT);
  //        cv::absdiff(bgrSplit.at(2),bgrSplit.at(1),rmgabs);
  //        cv::threshold(rmgabs,rmgabs,rgThresh,255,THRESH_BINARY);
  //        cv::bitwise_and(rlg,rmgabs,rg);
  //        //all region &

  Mat target;
  //        cv::bitwise_and(rb,rg,target);
  //        cv::bitwise_and(target,hsvbin,target);
  // cv::imshow("bin",target);
  target= hsvbin;
  //     cv::imshow("bin",target);
  // count number of red point
  int num= 0;
  for(int i= 0; i < target.rows; i++)
  {
    for(int j= 0; j < target.cols; j++)
    {
      if(int(target.at<uchar>(i, j)) > 100)
        num++;
    }
  }

  int tx= 0;
  int ty= 0;
  int num_b= 0;
  for(int i= 0; i < target.rows; i++)
  {
    for(int j= 0; j < target.cols; j++)
    {
      if(int(target.at<uchar>(i, j)) > 100)
      {
        tx= tx + j;
        ty= ty + i;
        num_b++;
      }
    }
  }
  tx= tx / (num_b + 0.00001);
  ty= ty / (num_b + 0.00001);

  cout << "tx = " << tx << endl;
  if(tx < 300)
    direction= -1;
  else if(tx > 340)
    direction= 1;
  else
    direction= 0;
  cout << "direction =" << direction << endl;

  //        if(num>10000)
  //                redLineFound=true;
  //        else
  //                redLineFound=false;
  //         cout<<"red points number  is:"<<num<<"red line found
  //         is:"<<redLineFound<<endl;
}
