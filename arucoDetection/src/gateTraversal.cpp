#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class gateTraversal{

 private:
    ros::Subscriber sub_pose;
    ros::Publisher pub;
    std::string pos_tag;
    

 public:
   ros::NodeHandle *nh;
   gateTraversal(){
   nh->subscribe("/camera/color/image_raw", 100, &gateTraversal::imageCallback, this);			// Callback for pose published by camera detection node
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    	std::string param;
    	ros::NodeHandle *nh1;
    	nh1->getParam("/marker/y", param);
    	if(double(param) > 2.2 && param != ""){
    		ROS_INFO_STREAM("Right");
    	}
    	else if(param != "" && double(param) > 2.2){
    		ROS_INFO_STREAM("Left");
    	}
    	else{
    		ROS_INFO_STREAM("Empty");
    	}
    }
};
    
int main(int argc, char ** argv) {

    ros::init(argc, argv, "gate_traversal_node");
    gateTraversal object();
    ros::spin();
}
