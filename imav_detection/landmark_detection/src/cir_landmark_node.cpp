#include <opencv2/opencv.hpp>
#include <iostream>
#include "cv_toolbox.h"
#include "config.h"
#include <ros/ros.h>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "std_msgs/Int32MultiArray.h"

using namespace std;
using namespace cv;

Mat src_img;
void image_cb(const sensor_msgs::ImageConstPtr& msg)
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
    int show_image = atoi(pd.getData("show_image").c_str());
    
    ros::init(argc, argv, "cir_landmark_detect");
    ros::NodeHandle nh_;   
    ros::Rate rate(50);    
    ros::Publisher landmark_pub=nh_.advertise<std_msgs::Int32MultiArray>("landmark_detection",10);
    ros::Subscriber image_sub_ = nh_.subscribe("imav/video_publish", 1, image_cb);
    
    double min_area = 100;

    vector<Point> contour_templateH;
    cir_loadTemplateContours(contour_templateH);
    
    Mat binary_img;

while(ros::ok)
    {
        ros::spinOnce();
        rate.sleep();
        int detect_index = -1;
        int success = 0;
        int center_x = 320;
        int center_y = 240;
        int rectarea = 0;
        int rectangle = 0;
	float label = 0;

        if(!src_img.empty())
        {
	    binary_img = detect_blue(src_img);
            medianBlur(binary_img,binary_img,5);
            vector<vector<Point>> contours;
            contours = FindContours_external(binary_img);
            vector<RotatedRect> rects(contours.size());

            for (int i = 0 ; i < contours.size(); i++)
                {
                    double area = contourArea(contours[i]);
                    rects[i] = minAreaRect(contours[i]);
                    Point2f center;
                    float radius;
                    minEnclosingCircle(contours[i], center, radius);
                    auto aspect_ratio = max(rects[i].size.width, rects[i].size.height) / min(rects[i].size.width, rects[i].size.height);
                    if(area > min_area)
                    {
                           if (aspect_ratio <1.2){
                               double tempSimiRate = matchShapes(contours[i], contour_templateH, CV_CONTOURS_MATCH_I3, 1.0);
                                if(tempSimiRate < 0.005)
                                {
				    circle(src_img, center, radius,Scalar(0, 255, 0),2);
				    success = 1;
				    center_x = (int)center.x;
				    center_y = (int)center.y;
				    rectarea = rectArea(rects[i]);
				    rectangle = (int)rects[i].angle;
				    detect_index = 1;
				    //DrawRotatedRect(src_img,rects[i],Scalar(0, 255, 0),2, area);
                                }
                            }
                    }
               }
	       if(detect_index == -1)
		{
			success = 0;
			center_x = 320;
			center_y = 240;
			rectarea = 0;
			rectangle = 0;
		}
	if(show_image == 1){
		//imshow("binary_imgs", binary_img);
		imshow("cir_landmark_img", src_img);
	}
	waitKey(1);
	std_msgs::Int32MultiArray landmark_msg;
            landmark_msg.data.push_back(success);
            landmark_msg.data.push_back(center_x);
            landmark_msg.data.push_back(center_y);
            landmark_msg.data.push_back(rectarea);
            landmark_msg.data.push_back(rectangle);
        
            //ROS_INFO("target parameter: success: %d, c_x: %d, c_y: %d, area: %d, angle: %d",landmark_msg.data[0],\
                    landmark_msg.data[1],landmark_msg.data[2],landmark_msg.data[3],landmark_msg.data[4]);
            landmark_pub.publish(landmark_msg);
        }
	if (char(waitKey(1) == 'q')){
            break;
        }
    }
return 0;
}
