#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <sys/time.h>
#include <iostream>
#include <cstdlib>
#include<cmath>
#include "config.h"

using namespace std;

int64_t getCurrentTime()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec/1000;
}

#define PI 3.1415926535898
sensor_msgs::NavSatFix global_pose;
void global_pose_cb(const sensor_msgs::NavSatFixConstPtr& msg)
{
  global_pose = *msg;
  //ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  //ROS_INFO("got status: %d, %d", (int)msg->armed, (int)msg->connected);
}
//get rel_alt
std_msgs::Float64 current_rel_alt;
void rel_alt_cb(const std_msgs::Float64::ConstPtr& msg)
{
  current_rel_alt= *msg;
  //ROS_INFO("got current_rel_alt: %f",current_rel_alt.data);
}

//get compass yaw
std_msgs::Float64 Uavyaw;
void yaw_cb(const std_msgs::Float64::ConstPtr& msg)
{
   Uavyaw= *msg;
  //ROS_INFO("got  Uavyaw: %f",Uavyaw.data);
}

std_msgs::Int32MultiArray box_info;
void boxdetect_cb(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
	box_info = *msg;
}

std_msgs::Int32MultiArray landmark_info;
void landmark_cb(const std_msgs::Int32MultiArray::ConstPtr& msg)
{ 
   landmark_info = *msg;
}

void print(const mavros_msgs::Waypoint& wp)
{
    ROS_INFO("gps:%f %f %f", wp.x_lat,wp.y_long,wp.z_alt);
    ROS_INFO("%d %d %d %d",wp.command,wp.frame,wp.autocontinue,wp.is_current);
    ROS_INFO("param:%f %f %f %f", wp.param1,wp.param2,wp.param3,wp.param4);
}

mavros_msgs::WaypointList current_waypoints;
void get_waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    current_waypoints = *msg;

    for (size_t i = 0; i < current_waypoints.waypoints.size(); i++)
    {
        ROS_INFO("WP %d",int(i));
        print(current_waypoints.waypoints[i]);
    }
    
}

bool is_samelocation(float target_lat, float target_lon, float current_lat, float current_lon)
{
	if(abs(target_lat - current_lat) < 0.000005 && abs(target_lon - current_lon) < 0.000005)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int main(int argc, char *argv[])
{
    int state = 0;
	int rate_hz = 20;
	int get_box = 0;
	int get_landmark = 0;

    ParameterReader pd;
	float takeoff_rel_alt = atof(pd.getData("takeoff_relative_altitude").c_str());

	float box_latitude = atof(pd.getData("mailbox.latitude").c_str());
	float box_longitude = atof(pd.getData("mailbox.longitude").c_str());
	float box_rel_alt = atof(pd.getData("mailbox.relative_altitude").c_str());
	float box_time_thresh = atof(pd.getData("mailbox.search_time_thresh").c_str());

	float land_latitude = atof(pd.getData("landing.latitude").c_str());
	float land_longitude = atof(pd.getData("landing.longitude").c_str());
	float land_rel_alt = atof(pd.getData("landing.relative_altitude").c_str());
	float land_time_thresh = atof(pd.getData("landing.search_time_thresh").c_str());
	float max_v = atof(pd.getData("max_v").c_str());
	

    float kp_x = atof(pd.getData("kp_x").c_str());
    float kp_y = atof(pd.getData("kp_y").c_str());
    
    float h_kp_x = atof(pd.getData("h_kp_x").c_str());
    float h_kp_y = atof(pd.getData("h_kp_y").c_str());
    float h_max_v = atof(pd.getData("h_max_v").c_str());
    
	float offset_y = atof(pd.getData("offset_y").c_str());

    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Subscriber get_waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 5, get_waypoints_cb);
    
    ros::Rate rate(rate_hz);
	
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                              ("mavros/state", 10, state_cb);

  	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("mavros/cmd/arming");
  	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("mavros/set_mode", 1);

  	ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_pose_cb);

  	global_pose.header.seq = 0;

  	ros::Subscriber rel_alt_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/rel_alt",1,rel_alt_cb);
    
	ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10); 

	ros::Subscriber yaw_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg",1,yaw_cb);

	ros::Subscriber landmark_sub = nh.subscribe<std_msgs::Int32MultiArray>("landmark_detection",1,landmark_cb);

	ros::Subscriber boxdetect_sub = nh.subscribe<std_msgs::Int32MultiArray>("box_detection",1,boxdetect_cb);

    ros::Publisher servo_pub = nh.advertise<std_msgs::UInt32>("servo_control",10);

	 while(ros::ok() && (!current_state.connected))
  	{
    	ros::spinOnce();
    	rate.sleep();
  	}

	while(ros::ok)
	{
		//ROS_INFO("PX4 Mode: %s", current_state.mode.c_str());
        geometry_msgs::TwistStamped velocity_tw;
        velocity_pub.publish(velocity_tw);
		if (current_state.mode=="OFFBOARD")
       		{
            	state = 1;
	    		break;
        	}
		ros::spinOnce();
        rate.sleep();
	}

    ros::ServiceClient arming_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  	mavros_msgs::CommandBool srv;
	srv.request.value = true;

	if(arming_cl.call(srv))
	{
		ROS_INFO("ARM send ok %d", srv.response.success);
		cout << "armed ok"<<endl;
	}
	else
	{
		ROS_ERROR("Failed arming or disarming");
		cout << "armed fail"<<endl;
	}

    ros::ServiceClient cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  	mavros_msgs::SetMode srv_setMode;

    geometry_msgs::TwistStamped velocity_tw ,vs_body_axis;
    velocity_tw.twist.linear.x = 0;
    velocity_tw.twist.linear.y = 0;
    velocity_tw.twist.linear.z = 0;

    int last_waypoint = current_waypoints.waypoints.size();
    float h_latitude = current_waypoints.waypoints[last_waypoint-2].x_lat;
    float h_longitude = current_waypoints.waypoints[last_waypoint-2].y_long;
    float z_relalt = current_waypoints.waypoints[last_waypoint-2].z_alt;

    ros::Time::init();
	ros::Time last_request;
    ros::Time state3_last_request;
	ros::Time state7_last_request;
	ros::Time state2_last_request;
	ros::Time state6_last_request;
	ros::Time state55_request;

    while(ros::ok)
    {
        if(box_info.data.size() != 0)
			{
				get_box = box_info.data[0];
				//cout<<"get box: "<<get_box<<endl;
            }
	if(landmark_info.data.size() != 0)
	{
		get_landmark = landmark_info.data[0];
	}

        if(state == 1)
        {
            srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
            srv_setMode.request.custom_mode = "AUTO.MISSION";

            if(cl.call(srv_setMode))
            {
                ROS_INFO("setmode send ok");
            }
            else
            {
                ROS_ERROR("Failed SetMode");
                return -1;
            }
            state = 2;
	    state2_last_request = ros::Time::now();
        }
        ros::Time state2_new_request=ros::Time::now();
        if(state == 2 && get_box == 1 && state2_new_request - state2_last_request > ros::Duration(20.0))
        {
            velocity_pub.publish(velocity_tw);
     		srv_setMode.request.custom_mode = "OFFBOARD";
     		cl.call(srv_setMode);
            state3_last_request=ros::Time::now();
     		state = 3;
        }

        if(state == 3)
		{	
			if(box_info.data.size() != 0)
			{
				get_box = box_info.data[0];
				if(get_box == 1)
				{	
					//double kp_x = 0.0008;
					//double kp_y = 0.0008;
					vs_body_axis.twist.linear.x = -1 * current_rel_alt.data * kp_x * (320 - box_info.data[1]);
					if(vs_body_axis.twist.linear.x >= max_v)
						vs_body_axis.twist.linear.x = max_v;
					if(vs_body_axis.twist.linear.x <= -max_v)
						vs_body_axis.twist.linear.x = -max_v;
			
					vs_body_axis.twist.linear.y = current_rel_alt.data * kp_y * (offset_y + 240 - box_info.data[2]);
					if(vs_body_axis.twist.linear.y >= max_v)
						vs_body_axis.twist.linear.y = max_v;
					if(vs_body_axis.twist.linear.y <= -max_v)
						vs_body_axis.twist.linear.y = -max_v;

					vs_body_axis.twist.linear.z = 0;

					if(abs(box_info.data[1]-320)<10)
					{
							vs_body_axis.twist.linear.x=0;
					}
						if(abs(box_info.data[2]-(240+offset_y))<10)
					{
							vs_body_axis.twist.linear.y=0;
					} 
					velocity_tw.twist.linear.z = vs_body_axis.twist.linear.z;
					velocity_tw.twist.linear.x = vs_body_axis.twist.linear.x * cos(Uavyaw.data*PI/180.0)  + vs_body_axis.twist.linear.y * sin(Uavyaw.data*PI/180.0);
					velocity_tw.twist.linear.y = -vs_body_axis.twist.linear.x * sin(Uavyaw.data*PI/180.0) + vs_body_axis.twist.linear.y * cos(Uavyaw.data*PI/180.0);
					if(abs(box_info.data[1]-320)<10 && abs(box_info.data[2]-(240+offset_y))<10)
					{
						state = 4;
						last_request=ros::Time::now();
					}
				}
				if(get_box == 0)
				{
				velocity_tw.twist.linear.z = 0.0;
				//velocity_tw.twist.linear.x = 0.3*cos(getCurrentTime()/1000%360);
				//velocity_tw.twist.linear.y = 0.3*sin(getCurrentTime()/1000%360); 
				velocity_tw.twist.linear.x = 0.0;
				velocity_tw.twist.linear.y = 0.0;                
				}
			}
			
			velocity_pub.publish(velocity_tw);

			ros::Time state3_new_request=ros::Time::now();
			if(state3_new_request - state3_last_request > ros::Duration(30.0))
				state = 4;
			last_request=ros::Time::now();
		}

        if(state == 4)
        {
                        ros::ServiceClient cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  			mavros_msgs::SetMode srv_setMode;
  			srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
  			srv_setMode.request.custom_mode = "AUTO.LAND";	
		  	if(cl.call(srv_setMode)) 
  			{
    			ROS_INFO("setmode send ok");
  			}
  			else
  			{
    			ROS_ERROR("Failed SetMode");
    		return -1;
  			}
			state = 101; 
       }
       if(state == 101)
       {
            
			//velocity_tw.twist.linear.x = 0;
            //velocity_tw.twist.linear.y = 0;
			//velocity_tw.twist.linear.z = -0.5;
			//velocity_pub.publish(velocity_tw);
             ros::Time now_request = ros::Time::now();
			if(now_request - last_request > ros::Duration(30.0) || current_rel_alt.data <= 3)
            {
                std_msgs::UInt32 servo_control_flag;
                servo_control_flag.data = 1;
                servo_pub.publish(servo_control_flag);
				state = 5;
            }
        }

        if(state == 5)
        {
            
            ros::ServiceClient client_wp = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

			mavros_msgs::WaypointPush srv_wp;
			mavros_msgs::CommandHome set_home_srv;

			ros::ServiceClient cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
			mavros_msgs::SetMode srv_setMode;

			mavros_msgs::Waypoint wp;

			wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
			wp.is_current   = true;
  			wp.autocontinue = true;
  			wp.param1       = 5;
  			wp.param4       =NAN;
  			wp.z_alt        = z_relalt;
  			wp.x_lat        = global_pose.latitude;
  			wp.y_long       = global_pose.longitude;
  			srv_wp.request.waypoints.push_back(wp);

			wp.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  			wp.is_current   = false;
  			wp.autocontinue = true;
  			wp.param1       = 3;
			wp.param4       =NAN;
  			wp.z_alt        = z_relalt;
  			wp.x_lat        = h_latitude;
  			wp.y_long       = h_longitude;
  			srv_wp.request.waypoints.push_back(wp);

            if (client_wp.call(srv_wp))
			{
				// ok, check srv.response
				ROS_INFO("Uploaded WPs!");
				mavros_msgs::State current_state;
			}
			else
			{
				// roscpp error
				ROS_ERROR("Upload Failed");
			}

			srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
  			srv_setMode.request.custom_mode = "AUTO.MISSION";

			if(cl.call(srv_setMode))
			{
				ROS_INFO("setmode send ok");
			}
			else
			{
				ROS_ERROR("Failed SetMode");
				return -1;
			}
			state = 6;
			state6_last_request = ros::Time::now();
        }
	
	ros::Time state6_new_request=ros::Time::now();
	
	
	if(state == 6 && is_samelocation(h_latitude, h_longitude, global_pose.latitude, global_pose.longitude) && get_landmark == 0)
		{
			velocity_pub.publish(velocity_tw);
     		srv_setMode.request.custom_mode = "OFFBOARD";
     		cl.call(srv_setMode);
			
			state = 55;	
			state55_request=ros::Time::now();
		}
		if (state == 55)
		{
			ros::Time now_request = ros::Time::now();

			
			if(now_request - state55_request < ros::Duration(5.0))
			{
				velocity_tw.twist.linear.x = 1.0;
				velocity_tw.twist.linear.y = 0.0; 
			}
			if(now_request - state55_request >= ros::Duration(5.0) && now_request - state55_request < ros::Duration(11.0))
			{
				velocity_tw.twist.linear.x = 0.0;
            	velocity_tw.twist.linear.y = 1.0;
			}
			if(now_request - state55_request >= ros::Duration(11.0) && now_request - state55_request < ros::Duration(15.0))
			{
				velocity_tw.twist.linear.x = -1.0;
            	velocity_tw.twist.linear.y = 0.0;
			}
			if(now_request - state55_request >= ros::Duration(15.0) && now_request - state55_request < ros::Duration(18.0))
			{
				velocity_tw.twist.linear.x = 0.0;
            	velocity_tw.twist.linear.y = -1.0;
			}
			if(now_request - state55_request >= ros::Duration(18.0) && now_request - state55_request < ros::Duration(24.0))
			{
				velocity_tw.twist.linear.x = 1.0;
            	velocity_tw.twist.linear.y = 0.0;
			}
			if(now_request - state55_request >= ros::Duration(24.0) && now_request - state55_request < ros::Duration(27.0))
			{
				velocity_tw.twist.linear.x = 0.0;
            	velocity_tw.twist.linear.y = 1.0;
			}
			if(now_request - state55_request >= ros::Duration(27.0) && now_request - state55_request < ros::Duration(30.0))
			{
				velocity_tw.twist.linear.x = -1.0;
            	velocity_tw.twist.linear.y = 0.0;
			}
			
			if(now_request - state55_request >= ros::Duration(30.0))
            {
				state = 8;
            }

			if (get_landmark == 1)
			{
				state = 6;
			}

			velocity_tw.twist.linear.z = 0.0;
			velocity_pub.publish(velocity_tw);
		}
		
        if(state == 6 && current_rel_alt.data > 5.5 && get_landmark == 1 && state6_new_request - state6_last_request > ros::Duration(20.0))
		{
			velocity_pub.publish(velocity_tw);
     		srv_setMode.request.custom_mode = "OFFBOARD";
     		cl.call(srv_setMode);
     		state = 7;
				state7_last_request=ros::Time::now();
		}

		if(state == 7)
		{
			if(landmark_info.data.size() != 0)
			{
				get_landmark = landmark_info.data[0];
				if(get_landmark == 1)
				{
					//double kp_x = 0.0008;
					//double kp_y = 0.0008;
					vs_body_axis.twist.linear.x = -1 * current_rel_alt.data * h_kp_x * (320 - landmark_info.data[1]);
					if(vs_body_axis.twist.linear.x >= h_max_v)
						vs_body_axis.twist.linear.x = h_max_v;
					if(vs_body_axis.twist.linear.x <= -h_max_v)
						vs_body_axis.twist.linear.x = -h_max_v;
			
					vs_body_axis.twist.linear.y = current_rel_alt.data * h_kp_y * (offset_y+240 - landmark_info.data[2]);
					if(vs_body_axis.twist.linear.y >= h_max_v)
						vs_body_axis.twist.linear.y = h_max_v;
					if(vs_body_axis.twist.linear.y <= -h_max_v)
						vs_body_axis.twist.linear.y = -h_max_v;

					vs_body_axis.twist.linear.z = 0;

					if(abs(landmark_info.data[1]-320)<10)
					{
							vs_body_axis.twist.linear.x=0;
					}
						if(abs(landmark_info.data[2]-(240+offset_y))<10)
					{
							vs_body_axis.twist.linear.y=0;
					} 
					velocity_tw.twist.linear.z = vs_body_axis.twist.linear.z;
					velocity_tw.twist.linear.x = vs_body_axis.twist.linear.x * cos(Uavyaw.data*PI/180.0)  + vs_body_axis.twist.linear.y * sin(Uavyaw.data*PI/180.0);
					velocity_tw.twist.linear.y = -vs_body_axis.twist.linear.x * sin(Uavyaw.data*PI/180.0) + vs_body_axis.twist.linear.y * cos(Uavyaw.data*PI/180.0);
					if(abs(landmark_info.data[1]-320)<10 && abs(landmark_info.data[2]-(240+offset_y))<10)
						state = 8;	
				}
				if(get_landmark == 0)
				{
				velocity_tw.twist.linear.z = 0.0;
				//velocity_tw.twist.linear.x = 0.3*cos(getCurrentTime()/1000%360);
				//velocity_tw.twist.linear.y = 0.3*sin(getCurrentTime()/1000%360);
				velocity_tw.twist.linear.x = 0.0;
				velocity_tw.twist.linear.y = 0.0;
				}
			}
			
			ros::Time state7_new_request=ros::Time::now();
			if(state7_new_request - state7_last_request > ros::Duration(30.0))
				state = 8;
			velocity_pub.publish(velocity_tw);	
		}

		if(state == 8)
		{
			ros::ServiceClient cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  			mavros_msgs::SetMode srv_setMode;
  			srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
  			srv_setMode.request.custom_mode = "AUTO.LAND";	
		  	if(cl.call(srv_setMode)) 
  			{
    			ROS_INFO("setmode send ok");
  			}
  			else
  			{
    			ROS_ERROR("Failed SetMode");
    		return -1;
  			}
			state = 9; 
		}
		ROS_INFO("state %d",state);
		ros::spinOnce();
    	rate.sleep();  
    }
    return 0;
}
