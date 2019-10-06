#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandCode.h>
#include <iostream>
#include <geometry_msgs/Vector3.h> //三个float64
#include <serial/serial.h>//control pump

using namespace std;

#define CHANNEL_INIT -1
#define LOW 0
#define MID 1
#define HIGH 2


mavros_msgs::RCIn rcIn;
mavros_msgs::RCOut rcOut;

void get_rc_out(const mavros_msgs::RCOut::ConstPtr &msg)
{
    rcOut = *msg;
}

int rc_flag = 0;
int channel6_laststatus = CHANNEL_INIT;
int channel6_currentstatus = CHANNEL_INIT;

void checkChannel6()
{
	//cout << rcIn.channels[6] <<endl;
    if (rcIn.channels[6] > 1600)
    {
        channel6_currentstatus = HIGH;
    }
    else
    {
        channel6_currentstatus = LOW;
    }
    if (abs(rcIn.channels[6] - 1514) < 50)
    {
        channel6_currentstatus = CHANNEL_INIT;
    }
    if (channel6_laststatus == LOW && channel6_currentstatus == HIGH)
    {
	rc_flag = 1;
    }
    channel6_laststatus = channel6_currentstatus;
}

void get_rc_in(const mavros_msgs::RCIn::ConstPtr &msg)
{
    rcIn = *msg;
    checkChannel6(); 
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc_servo_control");
    ros::NodeHandle nh;
    
    ros::Publisher rc_servo_pub = nh.advertise<std_msgs::UInt32>("rc_servo_control",10);
    ros::Subscriber rc_out_sub = nh.subscribe<mavros_msgs::RCOut>("mavros/rc/out", 1, get_rc_out);
    ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 1, get_rc_in);
    ros::Rate rate(50);
    
    while (ros::ok() )
    {
	//cout << "last" <<channel6_laststatus<<endl;
	//cout << "current" <<channel6_currentstatus<<endl;
	
	if(rc_flag == 1)
	{
		std_msgs::UInt32 servo_control_flag;
                servo_control_flag.data = 1;
                rc_servo_pub.publish(servo_control_flag);
	}
	ros::spinOnce();
        rate.sleep();
    }

    
    return 0;
}
