#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <stdlib.h>
using namespace std;
/*************************************************************************/
/*         函数名：getRectSide										     */
/*		   功能：获取以点(x,y)为中心，2r为边长的矩形边缘点（顺时针方向）
 */
/*				结果储存在引用参数 side中								*/
/*				获取成功时返回true，超出边界时返回false					*/
/*				if_debug设为true时，会在原图上绘制值为120点，注意：会改变原图*/
/*************************************************************************/
bool getRectSide(Mat& src, vector<uchar>& side, int x, int y, int r,
                 bool if_debug= false)
{
  if((x >= r) && ((x + r) < src.cols) && (y >= r) &&
     ((y + r) < src.rows))  //是否超边界
  {
    uchar* data= src.ptr<uchar>(y - r);  //获取边缘点值
    for(int k= x - r; k < x + r; ++k)
    {
      side.push_back(*(data + k));
      if(if_debug)
        if(*(data + k) != 255)
          *(data + k)= 120;
    }
    for(int k= y - r + 1; k < y + r; ++k)
    {
      data= src.ptr<uchar>(k);
      side.push_back(*(data + x + r));
      if(if_debug)
        if(*(data + x + r) != 255)
          *(data + x + r)= 120;
    }
    data= src.ptr<uchar>(y + r);
    for(int k= x + r - 1; k >= x - r; --k)
    {
      side.push_back(*(data + k));
      if(if_debug)
        if(*(data + k) != 255)
          *(data + k)= 120;
    }
    for(int k= y + r - 1; k > y - r; --k)
    {
      data= src.ptr<uchar>(k);
      side.push_back(*(data + x - r));
      if(if_debug)
        if(*(data + x - r) != 255)
          *(data + x - r)= 120;
    }
    return true;
  }
  else
    return false;
}
/***************************************************************************/
/*		函数名：isTri														*/
/*		功能：判断点(x,y)周围r距离是否有三条边								*/
/*			有返回true，没有返回false										*/
/*			调试时输出边缘相对于左上角的路径长度							*/
/*		适用条件：边内部没有空隙											*/
/***************************************************************************/
bool isTri(Mat& src, int x, int y, int r, bool if_debug= false)
{
  int plus_sum= 0, minus_sum= 0;
  vector<uchar> sides;
  if(getRectSide(src, sides, x, y, r, if_debug))
  {
    for(int k= 0; k < (int)sides.size() - 1; k++)
      if((sides[k + 1] - sides[k]) == 255)
      {
        plus_sum++;
        if(if_debug)
          cout << "side position:" << k << " +" << plus_sum << endl;
      }
      else if((sides[k + 1] - sides[k]) == -255)
      {
        minus_sum++;
        if(if_debug)
          cout << "side position:" << k << " -" << minus_sum << endl;
      }

    if(plus_sum == 3 && minus_sum == 3)
      return true;
    return false;
  }
  else
    return false;
}
/*****************************************************************/
/*		函数名：hasTri										 */
/*		功能：判断图片中是否有点周围r距离有三条边				 */
/*		要求：输入高斯滤波后的图和图中最大值					 */
/*		原理：随机选取100个处于(0.96val_max,val_max]的点		 */
/*			若其中有点周围有三条边，就判断有T型					 */
/*****************************************************************/
bool hasTri(Mat& src, int r, int val_max, bool if_debug= false)
{
  int x, y;
  Mat img_1, img_2;
  srand(val_max);
  threshold(src, img_2, val_max / 2, 255,
            THRESH_BINARY);  //获取二值图，便于使用if_Tri函数
  threshold(src, img_1, val_max * 0.96, 255, THRESH_BINARY);
  for(int i= 0; i < 100; i++)
  {
    x= rand() % src.cols;
    y= rand() % src.rows;
    if(*(img_1.ptr(y) + x))
    {
      if(isTri(img_2, x, y, r, if_debug))
        return true;
    }
    else
      --i;
  }
  return false;
}
/***********************************************************************/
/*         最小二乘法类													*/
/*			输入坐标数组，得到两种直线方程y=kx+b, x=ky+b				*/
/*			根据两种方程的误差，选择较小的方程，并以此为之后计算的方程	*/
/*			计算中心点到直线的距离向量									*/
/************************************************************************/
class LeastSquare
{
  float a, b, ah, bh;  //斜率a, 截距b
public:
  bool is_kxb;
  float tx, ty;
  LeastSquare(
      const vector<int>& x,
      const vector<int>& y)  //构造函数，输入x，y坐标，得到斜率和截距
  {
    float t1= 0, t2= 0, t3= 0, t4= 0, t5= 0;
    for(int i= 0; i < (int)x.size(); ++i)
    {
      t1+= x[i] * x[i];
      t2+= x[i];
      t3+= x[i] * y[i];
      t4+= y[i];
      t5+= y[i] * y[i];
    }
    a= (t3 * x.size() - t2 * t4) / (t1 * x.size() - t2 * t2);
    // b = (t4 - a*t2) / x.size();
    b= (t1 * t4 - t2 * t3) / (t1 * x.size() - t2 * t2);

    // t1=0, t2=0, t3=0, t4=0;
    /*for(int i=0; i<(int)x.size(); ++i)
    {
        t1 += y[i]*y[i];
        t2 += y[i];
        t3 += y[i]*x[i];
        t4 += x[i];
    }  */
    ah= (t3 * x.size() - t2 * t4) / (t5 * x.size() - t4 * t4);
    // b = (t4 - a*t2) / x.size();
    bh= (t5 * t2 - t4 * t3) / (t5 * x.size() - t4 * t4);
    float error1= error_plan1(x, y), error2= error_plan2(x, y);
    is_kxb= error1 < error2;  //比较误差，选择较小的方程
    // error = is_kxb?error1:error2;
    // error = min(error1,error2);
    // avg = error/x.size();

    if(is_kxb)
    {
      tx= -1;
      ty= -a;
    }
    else
    {
      tx= -ah;
      ty= -1;
    }
    float length= sqrt(tx * tx + ty * ty);
    tx= tx / length;
    ty= ty / length;
    if((-tx - ty) < 0)
    {
      tx= -tx;
      ty= -ty;
    }
  }

  float getY(const float x) const  //计算函数值y=kx+b
  {
    return a * x + b;
  }

  float getX(const float y) const  //计算x值 x=ah*y+bh
  {
    return ah * y + bh;
  }

  float
  error_plan1(const vector<int>& x,
              const vector<int>&
                  y)  // y=kx+b方程误差计算（点到直线距离平方和）
  {
    float error= 0.0;
    for(int i= 0; i < (int)x.size(); ++i)
    {
      error+=
          (a * x[i] + b - y[i]) * (a * x[i] + b - y[i]) / (a * a + 1);
    }
    return error;
  }

  float error_plan2(const vector<int>& x,
                    const vector<int>& y)  // x=ah*y+bh方程误差计算
  {
    float error= 0.0;
    for(int i= 0; i < (int)x.size(); ++i)
    {
      error+= (ah * y[i] + bh - x[i]) * (ah * y[i] + bh - x[i]) /
              (ah * ah + 1);
    }
    return error;
  }

  float error_point(float& x, float& y)
  {
    if(is_kxb)
      return (a * x + b - y) * (a * x + b - y) / (a * a + 1);
    else
      return (ah * y + bh - x) * (ah * y + bh - x) / (ah * ah + 1);
  }

  void print() const  //显示直线方程
  {
    if(is_kxb)
      cout << "y = " << a << "x + " << b << "\n";
    else
      cout << "x = " << ah << "y + " << bh << "\n";
  }

  void draw(Mat& src)  //绘图
  {
    uchar* data;
    if(is_kxb)  //如果是y=kx+b误差较小
    {
      for(int i= 0; i < src.cols; ++i)  //在原图上绘制结果
      {
        int j= (int)getY(i);
        if(j >= 0 && j <= src.rows)
        {
          data= src.ptr<uchar>(j);
          *(data + i)= 255;
        }
      }
    }
    else  //如果是x=ah*y+bh误差较小
    {
      for(int i= 0; i < src.rows; ++i)  //在原图上绘制结果
      {
        data= src.ptr<uchar>(i);
        int j= (int)getX(i);
        if(j >= 0 && j <= src.cols)
        {
          *(data + j)= 255;
        }
      }
    }
  }

  void direction(float x, float y, float& distance_x,
                 float& distance_y)  //获取从点（x，y）到直线的向量
  {
    if(is_kxb)
    {
      distance_x= -a * (a * x + b - y) / (a * a + 1);
      distance_y= (a * x + b - y) / (a * a + 1);
    }
    else
    {
      distance_x= -(x - bh - ah * y) / (ah * ah + 1);
      distance_y= ah * (x - bh - ah * y) / (ah * ah + 1);
    }
  }
};

/************************************************************************************/
/*			函数名：getYellowRegion												*/
/*			功能：输入图像src，返回图像dst，黄色区域值为200,						*/
/*				其他区域根据满足条件多少，值分别为0, 50, 100, 150					*/
/*				便于显示直观图像调试												*/
/*				将最后一个参数if_debug赋1后，将通过颜色分别显示HSV三通道二值化结果
 */
/*				对应关系：蓝色对于H通道，绿色对应S通道，红色对应V通道 */
/************************************************************************************/
void getYellowRegion(Mat& src, Mat& dst, int h_low= 30,
                     int h_high= 75, int s_threshold= 80,
                     int v_threshold= 80, bool if_debug= 0)
{
  Mat hsv, Line_h1, Line_h2, Line_h, Line_sv;
  vector<Mat> hsvSplit;
  cvtColor(src, hsv, CV_BGR2HSV_FULL);  //转换成HSV
  split(hsv, hsvSplit);  //分离出HSV通道，用于提取黄色区域
  threshold(hsvSplit[0], Line_h1, h_low, 255,
            THRESH_BINARY);  //去除 H 小于 h_low 度的区域
  threshold(hsvSplit[0], Line_h2, h_high, 255,
            THRESH_BINARY_INV);  //去除 H 大于 h_high 度的区域
  threshold(hsvSplit[1], hsvSplit[1], s_threshold, 255,
            THRESH_BINARY);  //去除 S 小于 s_threshold 的区域
  threshold(hsvSplit[2], hsvSplit[2], v_threshold, 255,
            THRESH_BINARY);  //去除 V 小于 v_threshold 的区域
  bitwise_and(Line_h1, Line_h2, Line_h);
  bitwise_and(hsvSplit[1], hsvSplit[2], Line_sv);
  bitwise_and(Line_h, Line_sv, dst);  //矩阵位与（255&255=255）
  // Line_h = Line_h1 + Line_h2;
  // dst = Line_h + hsvSplit[1] + hsvSplit[2];
  if(if_debug)  //调试用
  {
    hsvSplit[0]= Line_h;
    merge(hsvSplit, hsv);
    imshow("debug", hsv);
    waitKey(1);
  }
}
/*************************************************************************************/
/*		函数名：detectLine																 */
/*		功能：对输入图像拟合直线													 */
/*			距离向量结果储存在函数引用参数vector_x,vector_y中 */
/*			线的单位方向向量储存在引用参数Line_x,Line_y中							 */
/*			以竖直向上为x轴，水平向右为y轴											 */
/*			if_debug表示是否调试，默认不调试										 */
/*			调试时显示中间状况的窗口以及相关输出									 */
/*			不调试时，仅仅计算距离向量和方向向量并储存								 */
/*************************************************************************************/
void detectLine(Mat& src, float& distance_x, float& distance_y,
                float& line_vector_x, float& line_vector_y,
                bool if_debug= false)
{
  Mat img;
  vector<Mat> bgrSplit;
  vector<int> x, y;
  float picture_vector_x, picture_vector_y;  //图片参考系的距离向量
  uchar* data;  //获取图像数据所用数组
  if(if_debug)
    split(src, bgrSplit);  //分离出BGR通道，为最终显示结果做准备

  getYellowRegion(src, img, 25, 70, 70, 80, if_debug);  //获取黄色区域
  for(int i= 0; i < src.rows; ++i)  //遍历每一行
  {
    data= img.ptr<uchar>(i);          //获取此行开头指针
    for(int j= 0; j < src.cols; ++j)  //遍历此行每个元素
    {
      if(*data ==
         255)  //如果刚好满足之前4个条件（255 =255&255&255&255）
      {
        x.push_back(j);  //添加 x 坐标
        y.push_back(i);  //添加 y 坐标
      }
      ++data;  //指到下一个元素
    }
  }

  if(!x.empty())  //如果有数据
  {
    LeastSquare leastsq(x, y);  //拟合曲线
    leastsq.direction(
        src.cols / 2, src.rows / 2, picture_vector_x,
        picture_vector_y);  //获取中心点到直线的向量,图像坐标
    distance_y= picture_vector_x;  //转换为无人机坐标
    distance_x= -picture_vector_y;
    line_vector_y= leastsq.tx;
    line_vector_x= -leastsq.ty;
    if(if_debug)
    {
      leastsq.print();            //显示结果
      leastsq.draw(bgrSplit[1]);  //绘制图线
      detectLine(bgrSplit[1], Point(src.cols / 2, src.rows / 2),
                 Point(src.cols / 2 + picture_vector_x,
                       src.rows / 2 + picture_vector_y),
                 Scalar(255));  //以中心点为起点绘制该向量
      merge(bgrSplit, src);     //加入原图
    }
  }
}
/**************************************************************************/
/*		函数名：detectLineWithT													  */
/*		功能： 判断是否有T型，若有，返回true  							  */
/*			若无，返回false，并计算出中心点到直线的距离向量				  */
/*			距离向量结果储存在函数引用参数vector_x,vector_y中			  */
/*			线的单位方向向量储存在引用参数Line_x,Line_y中				  */
/*			以竖直向上为x轴，水平向右为y轴								  */
/*			if_debug表示是否调试，默认不调试							  */
/*			调试时显示中间状况的窗口以及相关输出						  */
/*			不调试时，仅仅计算距离向量和方向向量并储存					  */
/**************************************************************************/
bool detectLineWithT(Mat& src, float& distance_x, float& distance_y,
                     float& line_vector_x, float& line_vector_y,
                     bool if_debug= false)
{
  Mat img, T_img;  // img用于拟合，T_img用于判断
  vector<Mat> bgrSplit;
  vector<int> x, y;  // x，y坐标储存vector
  float picture_vector_x, picture_vector_y;  //图片参考系的距离向量
  int side= 71;    //判断T型的核边长大小
  double val_max;  //高斯滤波后的最大值
  Point p_max;     //高斯滤波后最大值的位置
  uchar* data;     //获取图像数据所用数组

  if(if_debug)
    split(src, bgrSplit);  //分离出BGR通道，为最终显示结果做准备
  getYellowRegion(src, img, 25, 70, 70, 80, if_debug);  //获取黄色区域
  Mat element1= getStructuringElement(
      MORPH_ELLIPSE, Size(5, 5));  //设置腐蚀的核大小,5x5的椭圆，即圆
  Mat element2= getStructuringElement(
      MORPH_ELLIPSE, Size(15, 15));  //设置膨胀的核大小
  erode(img, img, element1);         //腐蚀，去除噪点
  dilate(img, img, element2);  //膨胀，增加T型交叉点密度
  if(if_debug)
  {
    imshow("img", img);
    waitKey(1);
  }

  GaussianBlur(img, T_img, Size(side, side),
               0);  //高斯滤波，计算各点黄色密度
  minMaxLoc(T_img, NULL, &val_max, NULL,
            &p_max);  //得到密度最大点位置和值
  // cout
  // <<"test"<<hasTri(T_img,side,val_max,if_debug)<<endl;//另一种判断方式，测试功能
  // threshold(T_img, T_img, val_max/2, 255,
  // THRESH_BINARY);//二值化，便于判断是否有三条边

  // if(isTri(T_img, p_max.x, p_max.y, side,
  // if_debug))//判断最大点周围是否有三条边，若有，肯定为T型
  if(hasTri(T_img, side / 2, val_max, if_debug))
  {
    if(if_debug)
    {
      imshow("T_img", T_img);
      waitKey(1);
    }

    if(if_debug)
    {
      circle(bgrSplit[1], Point(p_max.x, p_max.y), side / 2,
             Scalar(255));
      merge(bgrSplit, src);
    }
    return true;
  }
  else  //不是T型，则计算距离向量
  {
    for(int i= 0; i < src.rows; ++i)  //遍历每一行
    {
      data= img.ptr<uchar>(i);          //获取此行开头指针
      for(int j= 0; j < src.cols; ++j)  //遍历此行每个元素
      {
        if(*data ==
           255)  //如果刚好满足之前4个条件（255 =255&255&255&255）
        {
          x.push_back(j);  //添加 x 坐标
          y.push_back(i);  //添加 y 坐标
        }
        ++data;  //指到下一个元素
      }
    }

    if(!x.empty())  //如果有数据
    {
      LeastSquare leastsq(x, y);  //拟合曲线
      leastsq.direction(
          src.cols / 2, src.rows / 2, picture_vector_x,
          picture_vector_y);  //获取中心点到直线的向量,图像坐标
      distance_y= picture_vector_x;  //转换为无人机坐标
      distance_x= -picture_vector_y;
      line_vector_y= leastsq.tx;
      line_vector_x= -leastsq.ty;
      if(if_debug)
      {
        leastsq.print();            //显示结果
        leastsq.draw(bgrSplit[1]);  //绘制图线
        detectLine(bgrSplit[1], Point(src.cols / 2, src.rows / 2),
                   Point(src.cols / 2 + picture_vector_x,
                         src.rows / 2 + picture_vector_y),
                   Scalar(255));  //以中心点为起点绘制该向量
        cout << "maxpoint:" << p_max.x << " " << p_max.y << endl;
        merge(bgrSplit, src);  //加入原图
      }
    }
    return false;
  }
}

int main()  //示例程序
{
  VideoCapture cap("line2.avi");  //获取视频
  // VideoCapture cap(1);
  Mat frame;  //定义用到的Mat
  float distance_x, distance_y, line_vector_x, line_vector_y;
  namedWindow("frame", 1);
  while(cap.read(frame))  //此函数读取图像并返回bool值，读取成功返回1
  {
    // frame = imread("detectLineWithT.png",3);
    if(detectLineWithT(frame, distance_x, distance_y, line_vector_x,
                       line_vector_y, false))
      cout << "T" << endl;
    else
    {
      cout << "distance:" << distance_x << " " << distance_y << endl;
      cout << "detectLine direction:" << line_vector_x << " "
           << line_vector_y << endl;
    }
    // detectLine(frame,distance_x,distance_y,line_vector_x,line_vector_y,true);
    // cout << distance_x<<" "<<distance_y<<endl;
    // cout << line_vector_x<<" "<<line_vector_y<<endl;
    imshow("frame", frame);
    // imwrite("result.png",frame);
    waitKey(0);
  }
  cap.release();
  return 0;
}
