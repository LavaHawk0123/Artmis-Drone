#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <opencv2/calib3d/calib3d.hpp> 
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <armadillo>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string> 
#include "geometry_msgs/Twist.h"
#include "iostream"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include <string>
#include <boost/geometry.hpp>

class GateTraversal{

 private:
    ros::Subscriber sub,sub_gps;
    ros::Publisher pub_vel;
    double yaw_set,yaw;
    bool tagLeft=0;
    bool tagRight=0;
    bool turn = 1;
    ros::Time curr_time;
    geometry_msgs::Twist Vel;
    double elapsed_time;
    double y;
    double Lat,Long;
    double lat_initial,long_initial;
    bool forward=0;
    

  public:
   GateTraversal(ros::NodeHandle *nh){
   sub_gps = nh->subscribe("/fix", 1000, &GateTraversal::gpsCallback, this); 
   pub_vel = nh->advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
   }
   
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    	   Lat = msg->latitude;
           Long = msg->longitude;
           ros::NodeHandle nh;
  	   nh.getParam("/marker/y",y);
  	   ROS_INFO_STREAM("Y Coordinate Got : "<<y);
	   if(y<-0.35 && y!=0){
	        ROS_INFO_STREAM("Left Tag Deteceted");
	   	tagLeft =1;
	   	Vel.angular.z =-0.1; 
	   }
	   else if(y>-0.35 && y!=0){
	        ROS_INFO_STREAM("Right Tag Deteceted");
	   	tagRight=1;
	  	Vel.angular.z =0.1; 
	   }
  	   if(tagLeft or tagRight){
  		ROS_INFO_STREAM("Marker Deteceted : "<<y);
  		//ros::Time start_time = ros::Time::now();
		/*while(turn){
			curr_time = ros::Time::now();
			elapsed_time = curr_time.toSec()  - start_time.toSec() ;
			if(elapsed_time<10){
				Vel.angular.z = 1;
				pub_vel.publish(Vel);
			}
			if(elapsed_time>=10){
				turn=0;
				forward=1;
				break;
				
			}
  		}*/
  	}
  	   else{
  	   	ROS_INFO_STREAM("Marker not Deteceted");
  	   }
  	   /*if(not turn){
  	   	move_forward();
  	   }*/
}
  
  void move_forward(){
  	if(forward){
	  	forward=0;
	  	lat_initial = Lat;
	  	long_initial = Long;
	  	ROS_INFO_STREAM(Lat);
  	}
  	float distance;
  	ros::NodeHandle nh;
  	nh.getParam("/marker/distance",distance);
  }
  
   
};
   


int main(int argc, char** argv)
{

  ros::init(argc, argv, "gate_traversal");
  ros::NodeHandle nh;
  GateTraversal obj =  GateTraversal(&nh);
  ros::spin();
}
