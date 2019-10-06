#include <ros/ros.h>
#include <ros/param.h>
#include <serial/serial.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include <iostream>
#include<std_msgs/ByteMultiArray.h>
#include<std_msgs/UInt32.h>
#include "config.h"

using namespace std;

serial::Serial sp; //声明串口对象
void write_cb(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
    sp.write(msg->data);   //发送串口数据 
}

std_msgs::UInt32 servo_control_flag;
void servo_cb(const std_msgs::UInt32ConstPtr& msg)
{
    servo_control_flag = *msg;
}

std_msgs::UInt32 rc_servo_control_flag;
void rc_servo_cb(const std_msgs::UInt32ConstPtr& msg)
{
    rc_servo_control_flag = *msg;
}


int main(int argc,char** argv)
{   
    ParameterReader pd;
    ros::init(argc,argv,"servo_control_node");
    ros::NodeHandle nh;
    ros::Time last_request;
    int flag=0;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_cb); 

    ros::Subscriber servo_sub = nh.subscribe("servo_control",1,servo_cb);
    ros::Subscriber rc_servo_sub = nh.subscribe("rc_servo_control",1,rc_servo_cb);

    try
    {
	sp.setPort(pd.getData("servo_port_name"));
        sp.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        sp.setTimeout(to);
        sp.open();
    }
    catch (serial::IOException& e)
    {
       ROS_ERROR_STREAM("Unable to open port "); 
       return -1;
    }
 
    if(sp.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 
  
    ros::Rate loop_rate(50); 
    while(ros::ok()) 
    {       
        uint8_t s_buffer[4];
        size_t sBUFFERSIZE=4;
        if(servo_control_flag.data == 1 || rc_servo_control_flag.data == 1)
        {
            s_buffer[0]=0xEB;
	    s_buffer[1]=0xFC;
	    s_buffer[2]=0x01; //01 servo open
	    s_buffer[3]=0xFE;
            sp.write(s_buffer,sBUFFERSIZE);
            //ROS_INFO_STREAM("servo open");
	    flag+=1;
        }
	if(flag==1)
	{last_request=ros::Time::now();}
	ros::Time now_request = ros::Time::now();
	if(now_request - last_request > ros::Duration(30.0) && flag>0)
	{
	    s_buffer[0]=0xEB;
	    s_buffer[1]=0xFC;
	    s_buffer[2]=0x02; //01 servo close
	    s_buffer[3]=0xFE;
            sp.write(s_buffer,sBUFFERSIZE);
            //ROS_INFO_STREAM("servo close");
	}

        ros::spinOnce(); 
        loop_rate.sleep(); 
    }    
}
