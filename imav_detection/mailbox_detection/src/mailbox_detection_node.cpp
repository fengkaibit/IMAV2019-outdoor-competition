#include <istream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Int32MultiArray.h"

#include "cvtoolbox.h"
#include "detect_color.h"
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

sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps = *msg;
    //ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);
}

int main(int argc,char **argv)
{
    int mailbox_color = atoi(pd.getData("mailbox_color").c_str());
    double red_min_area = atof(pd.getData("red_min_area").c_str());
    double blue_min_area = atof(pd.getData("blue_min_area").c_str());
    double yellow_min_area = atof(pd.getData("yellow_min_area").c_str());
    double max_aspect_ratio = atof(pd.getData("max_aspect_ratio").c_str());
    double min_area_rate = atof(pd.getData("min_area_rate").c_str());
    int show_image = atoi(pd.getData("show_image").c_str());
    int write_gps = atoi(pd.getData("write_gps").c_str());
    string gps_path = pd.getData("gps_path");
    
    double min_area = 1500;
    
    ofstream outfile;
    outfile.open(gps_path, ios::out);
    
    ros::init(argc, argv, "mailbox_detect");
    ros::NodeHandle nh;   
    ros::Rate rate(50);    
    ros::Publisher mailbox_pub=nh.advertise<std_msgs::Int32MultiArray>("box_detection",10);
    ros::Subscriber image_sub = nh.subscribe("imav/video_publish", 1, image_cb);
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1, get_gps_cb);
	    
    vector<Point> contour_templateH;
    loadTemplateContours(contour_templateH);
    
    //vector<Point> cir_contour_templateH;
    //cir_loadTemplateContours(cir_contour_templateH);

    while (ros::ok) {
        ros::spinOnce();
        rate.sleep();
        int detect_index = -1;
        int success = 0;
        int center_x = 0;
        int center_y = 0;
        int rectarea = 0;
        int rectangle = 0;

        Mat binary_img, src_clone;
        src_clone = src_img.clone();
        if(!src_img.empty())
        {
            switch(mailbox_color){
            case 0:
                binary_img = detect_red(src_clone);
		min_area = red_min_area;
                break;
            case 1:
                binary_img = detect_blue(src_clone);
		min_area = blue_min_area;
                break;
            case 2:
                binary_img = detect_yellow(src_clone);
		min_area = yellow_min_area;
                break;
            default:
                binary_img = detect_red(src_clone);
            }
            
            medianBlur(binary_img,binary_img,5);
            vector<vector<Point>> contours;
            contours = FindContours(binary_img);
            vector<RotatedRect> rects(contours.size());

            for (int i = 0 ; i < contours.size(); i++)
                {
                    double area = contourArea(contours[i]);
                    rects[i] = minAreaRect(contours[i]);

                    if(area > min_area)
                    {
                        auto aspect_ratio = max(rects[i].size.width, rects[i].size.height) / min(rects[i].size.width, rects[i].size.height);
                        if (aspect_ratio < max_aspect_ratio){
			    double tempSimiRate = matchShapes(contours[i], contour_templateH, CV_CONTOURS_MATCH_I3, 1.0);
			    //double cir_tempSimiRate = matchShapes(contours[i], cir_contour_templateH, CV_CONTOURS_MATCH_I3, 1.0);
			    double area_rate = area / rectArea(rects[i]);
                            if(tempSimiRate < 0.1 && area_rate > min_area_rate)
                            {
				    success = 1;
				    center_x = (int)rects[i].center.x;
				    center_y = (int)rects[i].center.y;
				    rectarea = rectArea(rects[i]);
				    rectangle = (int)rects[i].angle;
				    detect_index = 1;
				    if(show_image==1)
				    {
					DrawRotatedRect(src_clone,rects[i],Scalar(0, 255, 0),2, area);
				    }
				    if(write_gps==1)
				    {
					if(!outfile.is_open ())
						cout << "Open outfile failure" << endl;
					outfile << current_gps.longitude << ", "<< current_gps.latitude << ", mailbox_color: "<<mailbox_color<< endl;
				    }
			    }
                        }
                    }
                }
            if(detect_index == -1)
            {
                success = 0;
                center_x = 320; //usb_cam image center x
                center_y = 240; //usb_cam image center y
                rectarea = 0;
                rectangle = 0;
            }
            if(show_image==1){
                //imshow("binary_img", binary_img);
                imshow("mailbox_img", src_clone);
            }
            waitKey(1);
            std_msgs::Int32MultiArray mailbox_msg;
            mailbox_msg.data.push_back(success);
            mailbox_msg.data.push_back(center_x);
            mailbox_msg.data.push_back(center_y);
            mailbox_msg.data.push_back(rectarea);
            mailbox_msg.data.push_back(rectangle);
        
            //ROS_INFO("target parameter: success: %d, c_x: %d, c_y: %d, area: %d, angle: %d",mailbox_msg.data[0], \
                    mailbox_msg.data[1],mailbox_msg.data[2],mailbox_msg.data[3],mailbox_msg.data[4]);
            mailbox_pub.publish(mailbox_msg);    
        }
        if (char(waitKey(1) == 'q')){
            break;
        }
    }
    outfile.close();
    return 0;
}
