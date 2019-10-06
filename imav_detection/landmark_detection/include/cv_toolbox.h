#ifndef CV_TOOLBOX_H
#define CV_TOOLBOX_H

#include <opencv2/opencv.hpp>
#include <fstream>
#include <ostream>
#include <iostream>
#include "config.h"

using namespace std;
using namespace cv;

ParameterReader pd;

string H_txt_path = pd.getData("H_txt_path");
string cir_txt_path = pd.getData("cir_txt_path");
int min_diff_thresh = atoi(pd.getData("min_diff_thresh").c_str());
int show_min_diff = atoi(pd.getData("show_min_diff").c_str());
int binary_method = atoi(pd.getData("binary_method").c_str());


cv::Mat binarize(cv::Mat& rgbImg, const char* mode){
    cv::Mat biImg;
    if(strcmp(mode, "rgb")==0){
        cv::cvtColor(rgbImg, biImg, cv::COLOR_BGR2GRAY);
	cv::threshold(biImg, biImg, 150, 255, cv::THRESH_BINARY);
        //cv::threshold(biImg, biImg, 0, 255, cv::THRESH_OTSU);
    }
    else if(strcmp(mode, "hsv")==0){
        cv::Mat hsv;
        cv::cvtColor(rgbImg, hsv,cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> planes(3);
        cv::split(hsv, planes);
        cv::threshold(planes[1], biImg, 180, 255, cv::THRESH_OTSU);
    }
    //cv::bitwise_not(biImg, biImg);
    return biImg;
}

std::vector<std::vector<cv::Point> > FindContours(const cv::Mat &binary_img) {
  std::vector<std::vector<cv::Point> > contours;
  const auto mode = CV_RETR_TREE;
  //const auto mode = CV_RETR_EXTERNAL;
  const auto method = CV_CHAIN_APPROX_SIMPLE;
  cv::findContours(binary_img, contours, mode, method);
  return contours;
}

std::vector<std::vector<cv::Point> > FindContours_external(const cv::Mat &binary_img) {
  std::vector<std::vector<cv::Point> > contours;
  //const auto mode = CV_RETR_TREE;
  const auto mode = CV_RETR_EXTERNAL;
  const auto method = CV_CHAIN_APPROX_SIMPLE;
  cv::Mat binary_img_clone=binary_img.clone();
  cv::findContours(binary_img_clone, contours, mode, method);
  return contours;
}

void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) {
  cv::Point2f vertex[4];

  cv::Point2f center = rect.center;
  float angle = rect.angle;
  std::ostringstream ss;
  ss << angle;
  std::string text_angle(ss.str());

  std::string text = "angle is :" + text_angle;
  int font_face = cv::FONT_HERSHEY_COMPLEX;
  double font_scale = 0.5;
  cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

  rect.points(vertex);
  for (int i = 0; i < 4; i++)
    cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
}

void loadTemplates(string filename, std::vector<cv::Mat>& templates){
    std::ifstream in;
    in.open(filename, std::ios::in);
    if(in){
        std::string line;
        while(!in.eof()){
            getline(in, line);
            if(line.length()<1)
                continue;
            cv::Mat tmpl=cv::imread(line);
            if(tmpl.channels()==3){
                cv::cvtColor(tmpl, tmpl, cv::COLOR_BGR2GRAY);
                cv::threshold(tmpl,tmpl,150,255,CV_THRESH_BINARY);
		
		if(binary_method==0)
		  {cv::bitwise_not(tmpl, tmpl);}
            }
            templates.push_back(tmpl);
        }
        in.close();
    }
}

int knn(cv::Mat& img, std::vector<cv::Mat>& templates){
    int length=img.rows*img.cols;
    int min_diff=1e7;
    int index=-1;
    for(int i=0;i<templates.size();++i){
        cv::Mat& tmpl=templates[i];
        uchar* data=tmpl.data;
        uchar* img_data=img.data;
        int diff=0;
        for(int i=0;i<length;++i){
            if(data[i]!=img_data[i]){
                ++diff;
            }
        }
        if(diff<min_diff){
            min_diff=diff;
            index=1;
        }
    }
    //std::cout<<min_diff<<std::endl;
    if(min_diff>min_diff_thresh)
        index=-1;
    if(show_min_diff==1)
	cout<<"min diff: "<<min_diff<<endl;
    return index;
}

void loadTemplateContours(std::vector<cv::Point>& contour_templateH)
{
        ifstream file;
        file.open(H_txt_path,ios::in);
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

int rectArea(RotatedRect &rect)
{
    Point2f corner_point[4];
    rect.points(corner_point);
    int rect_width = floor(sqrt((corner_point[0].x-corner_point[1].x)*(corner_point[0].x-corner_point[1].x)+(corner_point[0].y-corner_point[1].y)*(corner_point[0].y-corner_point[1].y)));
    int rect_height = floor(sqrt((corner_point[1].x-corner_point[2].x)*(corner_point[1].x-corner_point[2].x)+(corner_point[1].y-corner_point[2].y)*(corner_point[1].y-corner_point[2].y)));
    int rect_area = rect_width * rect_height;
    return rect_area;
}

Mat detect_blue_hsv(Mat& src_img)
{
    Mat img_hsv;
    cvtColor(src_img, img_hsv, CV_BGR2HSV);
    vector<Mat> hsvSplit;
    split(img_hsv,hsvSplit);
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit,img_hsv);
    Mat img_hsv_blue;
    Mat img_threshold_blue;
    img_hsv_blue = img_hsv.clone();

    Mat blue_low(cv::Scalar(70, 150, 50));
    cv::Mat blue_higher(cv::Scalar(120, 255, 255));
    cv::inRange(img_hsv_blue, blue_low, blue_higher, img_threshold_blue);
    threshold(img_threshold_blue,img_threshold_blue,150,255,THRESH_BINARY);
    return img_threshold_blue;
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
    threshold(outputImage,outputImage,90,255,THRESH_BINARY);
    //adaptiveThreshold(outputImage,outputImage,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,0);
    return outputImage;
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
#endif // CV_TOOLBOX_H
