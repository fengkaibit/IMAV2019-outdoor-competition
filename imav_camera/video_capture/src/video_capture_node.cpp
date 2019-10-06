#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "config.h"

using namespace std;
using namespace cv;

Mat src_img;

void image_cb(const sensor_msgs::ImageConstPtr& msg)   //回调函数
{
    cv_bridge::CvImageConstPtr cv_ptr;  //申明一个CvImagePtr
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    src_img = cv_ptr->image.clone();
}

int main(int argc,char **argv)
{
    ParameterReader pd;
    string videonum_txt_path = pd.getData("videonum_txt_path");
    string save_video_path = pd.getData("save_video_path");
	int frame_rate = atoi(pd.getData("frame_rate").c_str());
    int width = atoi(pd.getData("width").c_str());
    int height = atoi(pd.getData("height").c_str());
    int show_image = atoi(pd.getData("show_image").c_str());
    
    ros::init(argc, argv, "video_capture");
    ros::NodeHandle nh;   
    ros::Rate rate(50);
    ros::Subscriber image_sub = nh.subscribe("imav/video_publish", 1, image_cb);

    int videonum;
    
    ifstream infile;
    ofstream outfile;
    infile.open(videonum_txt_path, ios::in);
    if(!infile.is_open ())
            cout << "Open infile failure" << endl;
    infile >> videonum;
    //保存视频的路径
    string outputVideoPath = save_video_path + to_string(videonum) + ".avi";
    videonum ++;
    infile.close();
    outfile.open(videonum_txt_path, ios::out);
    if(!outfile.is_open ())
            cout << "Open outfile failure" << endl;
    outfile << videonum << endl;
    outfile.close();
    //获取当前摄像头的视频信息
    cv::Size sWH = cv::Size(width, height);
    VideoWriter outputVideo;
    outputVideo.open(outputVideoPath, CV_FOURCC('M', 'J', 'P', 'G'), frame_rate, sWH);

    while (ros::ok)
    {
        ros::spinOnce();
        rate.sleep();
        if(!src_img.empty())
        {   
            if (src_img.empty()) break;
            outputVideo << src_img;
            if (show_image == 1)
            	imshow("img", src_img);
            if (char(waitKey(1) == 'q')){
                break;
            }   
        }
        else
        {
            cout <<"waiting image topic to start..."<< endl;
        }
           
    }

    outputVideo.release();
    return 0;
}
