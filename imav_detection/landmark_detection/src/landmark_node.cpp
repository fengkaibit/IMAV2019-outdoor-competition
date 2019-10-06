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

#define debug

#define PERSPECTIVE_TARGET  {cv::Point2f(0,64),\
                                cv::Point2f(0,0),\
                                cv::Point2f(64,0),\
                                cv::Point2f(64,64)}

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
    const string svm_path = pd.getData("svm_path");
    string H_temp_txt = pd.getData("H_temp_txt");
    double min_area = atof(pd.getData("min_area").c_str());
    double max_aspect_ratio = atof(pd.getData("max_aspect_ratio").c_str());
    int show_image = atoi(pd.getData("show_image").c_str());
    double temp_similar_thresh = atof(pd.getData("temp_similar_thresh").c_str());
    int canny_min = atoi(pd.getData("canny_min").c_str());
    int canny_max = atoi(pd.getData("canny_max").c_str());
    int detect_method = atoi(pd.getData("detect_method").c_str());
    
    Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    HOGDescriptor hog(Size(64, 64), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    if(detect_method == 0)
    {
	svm = cv::ml::SVM::load<ml::SVM>(svm_path);
	if (svm->empty())
	    cout << "load svm detector failed!!!" << endl;
	else
	    cout<<"load svm success!!!"<<endl;
    }
    vector<Point> contour_templateH;
    if(detect_method == 1)
    {
        loadTemplateContours(contour_templateH);
    }

    //cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::load<ml::SVM>(svm_path);

    ros::init(argc, argv, "landmark_detect");
    ros::NodeHandle nh_;   
    ros::Rate rate(50);    
    ros::Publisher landmark_pub=nh_.advertise<std_msgs::Int32MultiArray>("landmark_detection",10);
    ros::Subscriber image_sub_ = nh_.subscribe("imav/video_publish", 1, image_cb);

    vector<Mat> templates;
    loadTemplates(H_temp_txt,templates);

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

        Mat gray_img,binary_img, src_clone, H;
        vector<vector<Point> > contours;
        src_clone = src_img.clone();

        if(!src_img.empty())
        {
	    if(binary_method==0)
            {	
		cvtColor(src_clone,gray_img,CV_BGR2GRAY);
		char* mode = "rgb";
		binary_img = binarize(src_clone, mode);
	    }
	    if(binary_method==1)
		{binary_img = detect_blue_hsv(src_clone);}
            //threshold(gray_img,binary_img,180,255,CV_THRESH_OTSU);
            Mat canny_output;
            Canny(src_clone,canny_output,canny_min,canny_max,3);
            //if(show_image == 1)
                //imshow("canny",canny_output);

            contours = FindContours(canny_output);
            vector<RotatedRect> rects(contours.size());
            for(int i = 0; i < contours.size(); i++)
            {
                double area = contourArea(contours[i]);
                if(area > min_area)
                {
                    rects[i] = minAreaRect(contours[i]);
                    auto H_aspect_ratio =
                            std::max(rects[i].size.width, rects[i].size.height) / std::min(rects[i].size.width, rects[i].size.height);
                    auto angle = rects[i].angle;
                    Point center = rects[i].center;
                    Point2f corners[4];
                    rects[i].points(corners);
                    cv::Point2f perspective_target[4]=PERSPECTIVE_TARGET;

                    if (H_aspect_ratio < max_aspect_ratio && rects[i].size.area() >= min_area)
                    {
                        double tempSimiRate = matchShapes(contours[i], contour_templateH, CV_CONTOURS_MATCH_I3, 1.0);
                        //cout<<"temp"<<tempSimiRate<<endl;
                        if(tempSimiRate < temp_similar_thresh)
                        {
                            Rect boundRect = rects[i].boundingRect();
                            if (boundRect.x <= 0) { boundRect.x = 1; }
                            if (boundRect.x >= 639) { boundRect.x = 639; }
                            if (boundRect.y <= 0) { boundRect.y = 1; }
                            if (boundRect.y >= 479) { boundRect.y = 479; }
                            if (boundRect.x + boundRect.width > 639) { boundRect.width = 639 - boundRect.x; }
                            if (boundRect.y + boundRect.height > 479) { boundRect.height = 479 - boundRect.y; }

                            Mat warp;
                            Mat transform=cv::getPerspectiveTransform(corners,  perspective_target);
                            warpPerspective(binary_img, warp, transform, cv::Size(60, 60));
                            if(show_image == 1)
                                imshow("H",warp);

			    if(detect_method == 0)
			    {
                                vector<float> descriptors;
				hog.compute(warp,descriptors,Size(8,8));

				Mat testDescriptor = cv::Mat::zeros(1, descriptors.size(), CV_32FC1);
				for (size_t i = 0; i < descriptors.size(); i++)
				{
				    testDescriptor.at<float>(0, i) = descriptors[i];
				}
				label = svm->predict(testDescriptor);
			    }
			    if(detect_method == 1)
			    {
			        label = knn(warp, templates);
			    }
                            if(label > 0)
                            {
                                success = 1;
                                center_x = (int)rects[i].center.x;
                                center_y = (int)rects[i].center.y;
                                rectarea = rectArea(rects[i]);
                                rectangle = (int)rects[i].angle;
                                detect_index = 1;
                                if(show_image == 1)
                                    DrawRotatedRect(src_clone,rects[i],Scalar(0, 255, 0),2);
                            }              
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
            if(show_image == 1)
            {
                //imshow("binary_img",binary_img);
                imshow("landmark_img",src_clone);
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

