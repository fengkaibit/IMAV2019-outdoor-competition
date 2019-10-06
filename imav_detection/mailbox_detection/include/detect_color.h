#ifndef DETECT_COLOR_H
#define DETECT_COLOR_H

#include<opencv2/opencv.hpp>
#include <fstream>
#include <ostream>
#include "config.h"
using namespace cv;
using namespace std;

ParameterReader pd;

int red_threshold = atoi(pd.getData("red_threshold").c_str());
int blue_threshold = atoi(pd.getData("blue_threshold").c_str());
int yellow_threshold = atoi(pd.getData("yellow_threshold").c_str());
int H_min = atoi(pd.getData("H_min").c_str());
int S_min = atoi(pd.getData("S_min").c_str());
int V_min = atoi(pd.getData("V_min").c_str());
int H_max = atoi(pd.getData("H_max").c_str());
int S_max = atoi(pd.getData("S_max").c_str());
int V_max = atoi(pd.getData("V_max").c_str());
string rect_txt_path = pd.getData("rect_txt_path");
string cir_txt_path = pd.getData("cir_txt_path");

Mat detect_red(Mat& inputImage)
{
    int rowNumber = inputImage.rows;
    int colNumber = inputImage.cols;
    Mat outputImage;
    outputImage.create(rowNumber,colNumber,CV_8UC1);
    double r_div_g;
    const double e = 1;

    for(int i = 0;i < rowNumber;i++)
    {
        for(int j = 0;j < colNumber;j++)
        {
            if(inputImage.at<Vec3b>(i,j)[2] > inputImage.at<Vec3b>(i,j)[1])
            {
                r_div_g = double(100*(inputImage.at<Vec3b>(i,j)[2])/((inputImage.at<Vec3b>(i,j)[1])+e));  // R/G，找红色
                r_div_g = cvRound(r_div_g);
            }
            else
                r_div_g = 0;

        if(r_div_g>255)
            r_div_g=255;

         outputImage.at<uchar>(i,j) = r_div_g;
        }
    }
    threshold(outputImage,outputImage,red_threshold,255,THRESH_BINARY);
    //adaptiveThreshold(outputImage,outputImage,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,0);
    return outputImage;
}

Mat detect_blue(Mat& inputImage)
{
    int rowNumber = inputImage.rows;
    int colNumber = inputImage.cols;
    Mat outputImage;
    outputImage.create(rowNumber,colNumber,CV_8UC1);
    double b_div_g;
    const double e = 1;

    for(int i = 0;i < rowNumber;i++)
    {
        for(int j = 0;j < colNumber;j++)
        {
            if(inputImage.at<Vec3b>(i,j)[0] > inputImage.at<Vec3b>(i,j)[1] && inputImage.at<Vec3b>(i,j)[0] > inputImage.at<Vec3b>(i,j)[2])
            {
                b_div_g = double(100*(inputImage.at<Vec3b>(i,j)[0])/((inputImage.at<Vec3b>(i,j)[1])+e));  // B/G，找蓝色
                b_div_g = cvRound(b_div_g);
            }
            else
                b_div_g = 0;

        if(b_div_g>255)
            b_div_g=255;

         outputImage.at<uchar>(i,j) = b_div_g;
        }
    }
    threshold(outputImage,outputImage,blue_threshold,255,THRESH_BINARY);
    //adaptiveThreshold(outputImage,outputImage,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,0);
    return outputImage;
}

Mat detect_yellow(Mat& src_img)
{
    Mat img_hsv;
    cvtColor(src_img, img_hsv, CV_BGR2HSV);
    vector<Mat> hsvSplit;
    split(img_hsv,hsvSplit);
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit,img_hsv);
    Mat img_hsv_yellow;
    Mat img_threshold_yellow;
    img_hsv_yellow = img_hsv.clone();

    Mat yellow_low(cv::Scalar(H_min, S_min, V_min));
    cv::Mat yellow_higher(cv::Scalar(H_max, S_max, V_max));
    cv::inRange(img_hsv_yellow, yellow_low, yellow_higher, img_threshold_yellow);
    threshold(img_threshold_yellow,img_threshold_yellow,yellow_threshold,255,THRESH_BINARY);
    return img_threshold_yellow;
}

void loadTemplateContours(std::vector<cv::Point>& contour_templateH)
{
        ifstream file;
        file.open(rect_txt_path,ios::in);
        //作为输出文件打开
        while (1)
        {
            Point tempPt = Point(0,0);
            file >>tempPt.x >>tempPt.y;
            if(tempPt.x == 0 && tempPt.y == 0)
                break;
            contour_templateH.push_back(tempPt);
        }
}

void cir_loadTemplateContours(std::vector<cv::Point>& contour_templateH)
{
        ifstream file;
        file.open(cir_txt_path,ios::in);
        //作为输出文件打开
        while (1)
        {
            Point tempPt = Point(0,0);
            file >>tempPt.x >>tempPt.y;
            if(tempPt.x == 0 && tempPt.y == 0)
                break;
            contour_templateH.push_back(tempPt);
        }
}

#endif // DETECT_COLOR_H

