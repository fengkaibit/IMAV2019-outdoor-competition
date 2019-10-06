#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "config.h"

using namespace std;
using namespace cv;

void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle)
{
	CV_Assert(!inputIm.empty());
	Mat inputImg;
	inputIm.copyTo(inputImg);
	float radian = (float)(angle / 180.0 * CV_PI);
	int uniSize = (int)(max(inputImg.cols, inputImg.rows) * 1.414);
	int dx = (int)(uniSize - inputImg.cols) / 2;
	int dy = (int)(uniSize - inputImg.rows) / 2;
	copyMakeBorder(inputImg, tempImg, dy, dy, dx, dx, BORDER_CONSTANT);
	Point2f center((float)(tempImg.cols / 2), (float)(tempImg.rows / 2));
	Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);
	warpAffine(tempImg, tempImg, affine_matrix, tempImg.size());
	float sinVal = fabs(sin(radian));
	float cosVal = fabs(cos(radian));
	Size targetSize((int)(inputImg.cols * cosVal + inputImg.rows * sinVal),
					(int)(inputImg.cols * sinVal + inputImg.rows * cosVal));
	int x = (tempImg.cols - targetSize.width) / 2;
	int y = (tempImg.rows - targetSize.height) / 2;
	Rect rect(x, y, targetSize.width, targetSize.height);
	tempImg = Mat(tempImg, rect);
}

int main(int argc, char *argv[])
{
    ParameterReader pd;
    int port_name = atoi(pd.getData("port_name").c_str());
   
    ros::init(argc, argv, "video_publish");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("imav/video_publish", 1);

    cv::VideoCapture cap(port_name);

    if (!cap.isOpened())
    {
        ROS_INFO("open image device failed\n");
        return 1;
    }

    ROS_INFO("Video Capture Open Success ...");

    cv::Mat src_image, frame;
    sensor_msgs::ImagePtr msg;

    cap >> src_image;
    if (src_image.data)
    {
        ROS_INFO("Video Capture Read Success ...");
    }
    else
    {
        ROS_ERROR("Video Capture Read Failed ...");
    }

    ros::Rate loop_rate(60);
    ros::Time last_request = ros::Time::now();

    ROS_INFO("Video Capture Working ...");
    while (ros::ok())
    {
        cap >> src_image;
	//warpFfine(src_image, frame, 180);
        if (!src_image.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_image).toImageMsg();
            image_pub.publish(msg);
        }
        else
        {
            ROS_INFO("frame empty ...");
        }
        //ros::spinOnce();
        //loop_rate.sleep();
    }

    return 0;
}
