#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <stdlib.h>
using namespace std;

#include <ros/ros.h>
#include "std_msgs/String.h"

class LeastSquare;

class RMChallengeVision
{
  public:
    RMChallengeVision()
    {
    }
    RMChallengeVision( bool visable );
    enum COLOR_TYPE
    {
        RED,
        GREEN,
        BLUE
    };
    struct PILLAR_RESULT
    {
        int triangle[4];
        bool circle_found;
        Point2f circle_center;
        float radius;
    };
    /**p0 angle point,return the cosine of angle*/
    int detectPillar( Mat src, PILLAR_RESULT& pillar_result );
    float angle( Point pt1, Point pt2, Point pt0 );
    void detectTriangle( Mat src, Mat color_region, int triangle[4] );
    void detectPillarCircle( Mat src, Mat color_region, bool& circle_found,
                             Point2f& circle_center, float& rad );
    void extractColor( Mat src, COLOR_TYPE color, Mat& colorRegion );
    float imageToRealDistance( float imageLength, float imageDistance,
                               float realLength );
    float imageToHeight( float imageLength, float realLength );

    /**yellow line related functions*/
    void getYellowRegion( Mat& src, Mat& dst, int h_low = 30, int h_high = 75,
                          int s_threshold = 80, int v_threshold = 80 );
    void detectLine( Mat& src, float& distance_x, float& distance_y,
                     float& line_vector_x, float& line_vector_y );
    bool detectLineWithT( Mat& src, float& distance_x, float& distance_y,
                          float& line_vector_x, float& line_vector_y );
    bool getRectSide( Mat& src, vector< uchar >& side, int x, int y, int r );
    bool isTri( Mat& src, int x, int y, int r );
    bool hasTri( Mat& src, int r, int val_max );

    /**set visability*/
    void setVisability( bool visable );

  private:
    bool m_visable = true;
};

class LeastSquare
{
    float a, b, ah, bh;  //斜率a, 截距b
  public:
    bool is_kxb;
    float tx, ty;
    LeastSquare( const vector< int >& x,
                 const vector< int >& y )  //构造函数，输入x，y坐标，得到斜率和截距
    {
        float t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
        for ( int i = 0; i < ( int )x.size(); ++i )
        {
            t1 += x[i] * x[i];
            t2 += x[i];
            t3 += x[i] * y[i];
            t4 += y[i];
            t5 += y[i] * y[i];
        }
        a = ( t3 * x.size() - t2 * t4 ) / ( t1 * x.size() - t2 * t2 );
        // b = (t4 - a*t2) / x.size();
        b = ( t1 * t4 - t2 * t3 ) / ( t1 * x.size() - t2 * t2 );

        // t1=0, t2=0, t3=0, t4=0;
        /*for(int i=0; i<(int)x.size(); ++i)
        {
            t1 += y[i]*y[i];
            t2 += y[i];
            t3 += y[i]*x[i];
            t4 += x[i];
        }  */
        ah = ( t3 * x.size() - t2 * t4 ) / ( t5 * x.size() - t4 * t4 );
        // b = (t4 - a*t2) / x.size();
        bh = ( t5 * t2 - t4 * t3 ) / ( t5 * x.size() - t4 * t4 );
        float error1 = error_plan1( x, y ), error2 = error_plan2( x, y );
        is_kxb = error1 < error2;  //比较误差，选择较小的方程
        // error = is_kxb?error1:error2;
        // error = min(error1,error2);
        // avg = error/x.size();

        if ( is_kxb )
        {
            tx = -1;
            ty = -a;
        }
        else
        {
            tx = -ah;
            ty = -1;
        }
        float length = sqrt( tx * tx + ty * ty );
        tx = tx / length;
        ty = ty / length;
        if ( ( -tx - ty ) < 0 )
        {
            tx = -tx;
            ty = -ty;
        }
    }

    float getY( const float x ) const  //计算函数值y=kx+b
    {
        return a * x + b;
    }

    float getX( const float y ) const  //计算x值 x=ah*y+bh
    {
        return ah * y + bh;
    }

    float error_plan1(
        const vector< int >& x,
        const vector< int >& y )  // y=kx+b方程误差计算（点到直线距离平方和）
    {
        float error = 0.0;
        for ( int i = 0; i < ( int )x.size(); ++i )
        {
            error +=
                ( a * x[i] + b - y[i] ) * ( a * x[i] + b - y[i] ) / ( a * a + 1 );
        }
        return error;
    }

    float error_plan2( const vector< int >& x,
                       const vector< int >& y )  // x=ah*y+bh方程误差计算
    {
        float error = 0.0;
        for ( int i = 0; i < ( int )x.size(); ++i )
        {
            error += ( ah * y[i] + bh - x[i] ) * ( ah * y[i] + bh - x[i] ) /
                     ( ah * ah + 1 );
        }
        return error;
    }

    float error_point( float& x, float& y )
    {
        if ( is_kxb )
            return ( a * x + b - y ) * ( a * x + b - y ) / ( a * a + 1 );
        else
            return ( ah * y + bh - x ) * ( ah * y + bh - x ) / ( ah * ah + 1 );
    }

    void print() const  //显示直线方程
    {
        if ( is_kxb )
            cout << "y = " << a << "x + " << b << "\n";
        else
            cout << "x = " << ah << "y + " << bh << "\n";
    }

    void draw( Mat& src )  //绘图
    {
        uchar* data;
        if ( is_kxb )  //如果是y=kx+b误差较小
        {
            for ( int i = 0; i < src.cols; ++i )  //在原图上绘制结果
            {
                int j = ( int )getY( i );
                if ( j >= 0 && j <= src.rows )
                {
                    data = src.ptr< uchar >( j );
                    *( data + i ) = 255;
                }
            }
        }
        else  //如果是x=ah*y+bh误差较小
        {
            for ( int i = 0; i < src.rows; ++i )  //在原图上绘制结果
            {
                data = src.ptr< uchar >( i );
                int j = ( int )getX( i );
                if ( j >= 0 && j <= src.cols )
                {
                    *( data + j ) = 255;
                }
            }
        }
    }

    void direction( float x, float y, float& distance_x,
                    float& distance_y )  //获取从点（x，y）到直线的向量
    {
        if ( is_kxb )
        {
            distance_x = -a * ( a * x + b - y ) / ( a * a + 1 );
            distance_y = ( a * x + b - y ) / ( a * a + 1 );
        }
        else
        {
            distance_x = -( x - bh - ah * y ) / ( ah * ah + 1 );
            distance_y = ah * ( x - bh - ah * y ) / ( ah * ah + 1 );
        }
    }
};