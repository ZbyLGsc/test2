/*************************************************
Function:           FindArmorV & FindBase
Description:        垂直视野找出装甲板（飞机用）
Input:              src：源图像
Output:             armors：装甲板位置
Others:
*************************************************/
#define DRAW 1
#define PI 3.1415926535898
#define WE_RED 0
#define WE_BLUE 1
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include <iostream>
#include <vector>
using namespace std;

extern bool we_color; //red:0 blue:1

void FindArmorR(Mat src, vector<Point> &armors)
{
    if (src.channels() != 3)
        return;

    //灰度图
    Mat gray;
    cvtColor(src, gray, COLOR_RGB2GRAY);
    //二值化
    Mat bin;
    Mat sorted;
    gray.reshape(1, 1).copyTo(sorted);
    cv::sort(sorted, sorted, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);
    int binThresh = sorted.ptr<uchar>(0)[10000]; //
    // cout << binThresh << endl;
    threshold(gray, bin, binThresh, 255, THRESH_BINARY);

    Mat element2 = getStructuringElement(MORPH_CROSS, Size(2, 2));
    Mat element3 = getStructuringElement(MORPH_CROSS, Size(3, 3));
    //    腐蚀
    erode(bin, bin, element2, Point(-1, -1), 1);
    //    膨胀
    dilate(bin, bin, element3, Point(-1, -1), 1);
    //
    //    erode(bin,bin,NULL ,Point(-1,-1),1);
    //    imshow("bin",bin);
    //    vector< Point > armors;
    //imshow("bin", bin);
    vector<vector<Point> > contours;
    vector<RotatedRect> lights;
    findContours(bin, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    if (contours.size() > 0)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            //面积不能太大或太小
            int area = contourArea(contours.at(i));
            if (area > 100 || area < 25)
                continue;
            RotatedRect rec = minAreaRect(contours.at(i));
            //不能太靠边
            int border = 10;
            double recx = rec.center.x;
            double recy = rec.center.y;
            int srcH = (int)src.size().height;
            int srcW = (int)src.size().width;
            if (recx < border || recx > srcW - border || recy < border || recy > srcH - border)
                continue;
            //长宽比合适
            Size2f size = rec.size;
            double a = size.height > size.width ? size.height : size.width;
            double b = size.height < size.width ? size.height : size.width;
            if (a<10)
                continue;
            if (a / b > 5 || a / b < 1.3)
            {
                //cout<<a/b<<endl;
                continue;
            }
            //周围明显呈蓝色（或红色）

            Mat roi = src(Rect(recx - 7, recy - 7, 15, 15));
            Scalar avg;
            avg = mean(roi);
            // r[2]-b[0]>10
            if (!(avg.val[2] - avg.val[0] > 20))
            {
                //cvResetIamgeROI(src);
                continue;
            }
            //cvResetIamgeROI(src);
            if (DRAW)
            {
                Point2f vertices[4];
                rec.points(vertices);
                for (int i = 0; i < 4; ++i)
                    line(src, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2);
            }
            lights.push_back(rec);
        }
    }
    //    cout<<lights.size()<<endl;
    if (lights.size() > 1)
    {

        for (int i = 0; i < lights.size() - 1; i++)
        {
            for (int j = i + 1; j < lights.size(); j++)
            {
                Point2f pi = lights.at(i).center;
                Point2f pj = lights.at(j).center;
                double midx = (pi.x + pj.x) / 2;
                double midy = (pi.y + pj.y) / 2;
                Size2f sizei = lights.at(i).size;
                Size2f sizej = lights.at(j).size;
                double ai = sizei.height > sizei.width ? sizei.height : sizei.width;
                double aj = sizej.height > sizej.width ? sizej.height : sizej.width;
                double distance = sqrt((pi.x - pj.x) * (pi.x - pj.x) + (pi.y - pj.y) * (pi.y - pj.y));
                //灯条距离合适
                if (distance < 3 * ai || distance > 5 * ai || distance < 3 * aj|| distance > 5 * aj)
                    continue;

                
                //灯条中点连线与灯条夹角合适
                double angeli = lights.at(i).angle;
                double angelj = lights.at(j).angle;
                if (sizei.width < sizei.height)
                    angeli += 90;
                if (sizej.width < sizej.height)
                    angelj += 90;
                double doti = abs(cos(angeli * PI / 180) * (pi.x - pj.x) + sin(angeli * PI / 180) * (pi.y - pj.y)) / distance;
                double dotj = abs(cos(angelj * PI / 180) * (pi.x - pj.x) + sin(angelj * PI / 180) * (pi.y - pj.y)) / distance;
                if (doti > 0.3 || dotj > 0.3)
                    continue;

                // line(src, Point2f(pi.x, pi.y),Point2f(pj.x,pj.y), Scalar(0, 0, 255), 1);
                // imshow("distance", src);
                //                Mat roi = gray(Rect(midx-50,midy-5,10,10));
                //                Mat avg,sd;
                //                //normalize(roi,roi);
                //                roi-=(mean(roi)[0]-50);
                //                meanStdDev(roi,avg,sd);
                //                //sds(i)
                //                if (sd.at<double>(0,0)<0)
                //                    continue;
                //                if (avg.at<double>(0,0)<0)
                //                    continue;
                //                cout<<avg.at<double>(0,0)<<endl;
                armors.push_back(Point((int)midx, (int)midy));
            }
        }
    }
}
void FindArmorB(Mat src, vector<Point> &armors)
{
    if (src.channels() != 3)
        return;

    //灰度图
    Mat gray;
    cvtColor(src, gray, COLOR_RGB2GRAY);
    //二值化
    Mat bin;
    Mat sorted;
    gray.reshape(1, 1).copyTo(sorted);
    cv::sort(sorted, sorted, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);
    int binThresh = sorted.ptr<uchar>(0)[10000]; //
    // cout << binThresh << endl;
    threshold(gray, bin, binThresh, 255, THRESH_BINARY);

    Mat element = getStructuringElement(MORPH_CROSS, Size(2, 2));
    //    //    腐蚀
    erode(bin, bin, element, Point(-1, -1), 1);
    //    膨胀
    dilate(bin, bin, element, Point(-1, -1), 1);
    //
    //    erode(bin,bin,NULL ,Point(-1,-1),1);
    //    imshow("bin",bin);
    //    vector< Point > armors;
    // imshow("bin", bin);
    vector<vector<Point> > contours;
    vector<RotatedRect> lights;
    findContours(bin, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    if (contours.size() > 0)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            //面积不能太大或太小
            int area = contourArea(contours.at(i));
            if (area > 100 || area < 10)
                continue;
            RotatedRect rec = minAreaRect(contours.at(i));
            //不能太靠边
            int border = 10;
            double recx = rec.center.x;
            double recy = rec.center.y;
            int srcH = (int)src.size().height;
            int srcW = (int)src.size().width;
            if (recx < border || recx > srcW - border || recy < border || recy > srcH - border)
                continue;
            //长宽比合适
            Size2f size = rec.size;
            double a = size.height > size.width ? size.height : size.width;
            double b = size.height < size.width ? size.height : size.width;
            if (a / b > 5 || a / b < 1.3)
            {
                //cout<<a/b<<endl;
                continue;
            }
            //周围明显呈蓝色（或红色）

            Mat roi = src(Rect(recx - 7, recy - 7, 15, 15));
            Scalar avg;
            avg = mean(roi);
            // b[0]-r[2]>10
            if (!(avg.val[0] - avg.val[2] > 20))
            {
                //cvResetIamgeROI(src);
                continue;
            }
            //cvResetIamgeROI(src);
            if (DRAW)
            {
                Point2f vertices[4];
                rec.points(vertices);
                for (int i = 0; i < 4; ++i)
                    line(src, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2);
            }
            lights.push_back(rec);
        }
    }
    //    cout<<lights.size()<<endl;
    if (lights.size() > 1)
    {

        for (int i = 0; i < lights.size() - 1; i++)
        {
            for (int j = i + 1; j < lights.size(); j++)
            {
                Point2f pi = lights.at(i).center;
                Point2f pj = lights.at(j).center;
                double midx = (pi.x + pj.x) / 2;
                double midy = (pi.y + pj.y) / 2;
                Size2f sizei = lights.at(i).size;
                Size2f sizej = lights.at(j).size;
                double ai = sizei.height > sizei.width ? sizei.height : sizei.width;
                //                double b=sizei.height<sizei.width?sizei.height:sizei.width;
                double distance = sqrt((pi.x - pj.x) * (pi.x - pj.x) + (pi.y - pj.y) * (pi.y - pj.y));
                //灯条距离合适
                if (distance < 3 * ai || distance > 4.5 * ai)
                    continue;
                //灯条中点连线与灯条夹角合适
                double angeli = lights.at(i).angle;
                double angelj = lights.at(j).angle;
                if (sizei.width < sizei.height)
                    angeli += 90;
                if (sizej.width < sizej.height)
                    angelj += 90;
                double doti = abs(cos(angeli * PI / 180) * (pi.x - pj.x) + sin(angeli * PI / 180) * (pi.y - pj.y)) / distance;
                double dotj = abs(cos(angelj * PI / 180) * (pi.x - pj.x) + sin(angelj * PI / 180) * (pi.y - pj.y)) / distance;
                if (doti > 0.3 || dotj > 0.3)
                    continue;
                //                Mat roi = gray(Rect(midx-50,midy-5,10,10));
                //                Mat avg,sd;
                //                //normalize(roi,roi);
                //                roi-=(mean(roi)[0]-50);
                //                meanStdDev(roi,avg,sd);
                //                //sds(i)
                //                if (sd.at<double>(0,0)<0)
                //                    continue;
                //                if (avg.at<double>(0,0)<0)
                //                    continue;
                //                cout<<avg.at<double>(0,0)<<endl;
                armors.push_back(Point((int)midx, (int)midy));
            }
        }
        cv::waitKey(1);
    }
}

void FindBase(Mat image, bool &BaseFound, Point &BaseCenter)
{
    static Point armorcenter(0, 0), tmp1(0, 0), tmp2(0, 0), tmp(0, 0);
    static bool flag = false;
    vector<Point> armors;
    if (we_color == WE_RED)
    {
        FindArmorB(image, armors);
    }
    else if (we_color == WE_BLUE)
    {
        FindArmorR(image, armors);
    }

    double dis1, dis2;
    BaseFound = false;
    if (armors.size() > 1 && !flag)
        flag = true;
    if (flag)
    {
        switch (armors.size())
        {
        case 1:
            tmp = armors.at(0);
            dis1 = sqrt((tmp.x - tmp1.x) * (tmp.x - tmp1.x) + (tmp.y - tmp1.y) * (tmp.y - tmp1.y));
            dis2 = sqrt((tmp.x - tmp2.x) * (tmp.x - tmp2.x) + (tmp.y - tmp2.y) * (tmp.y - tmp2.y));
            if (dis1 < dis2 && dis1 < 10)
            {
                tmp2 += tmp - tmp1;
                tmp1 = tmp;
            }
            else if (dis2 < 10)
            {
                tmp1 += tmp - tmp2;
                tmp2 = tmp;
            }
            armorcenter.x = (tmp1.x + tmp2.x) / 2;
            armorcenter.y = (tmp1.y + tmp2.y) / 2;
            BaseFound = true;
            break;
        case 2:
            tmp1 = armors.at(0);
            tmp2 = armors.at(1);
            armorcenter.x = (tmp1.x + tmp2.x) / 2;
            armorcenter.y = (tmp1.y + tmp2.y) / 2;
            BaseFound = true;
            break;
        default:
            flag = false;
            for (int i=0;i<armors.size();i++)
            {
                circle(image, armors.at(i), 4, CV_RGB(0, 0, 255), -1);
            }
            break;
        }
    }
    if (BaseFound)
    {
        circle(image, armorcenter, 6, CV_RGB(255, 0, 0), -1);
        //drawContours(image, lights, -1, Scalar(0,255,0), CV_FILLED);
        //imshow("Video", image);
        BaseCenter = armorcenter;
    }
}
