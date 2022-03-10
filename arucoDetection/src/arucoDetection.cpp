#include "arucoDetection.h"

using namespace ad;

arucoDetection::arucoDetection() {

    MarkerPose.header.frame_id = "zed2_base_link";

    ROS_INFO_STREAM("Started Aruco Detection Node");
    pub_marker = nh.advertise < sensor_msgs::Image > ("/marker_detect/image_raw", 100);					// Publisher for Image of detection
    pub_pose_estimated = nh.advertise < sensor_msgs::Image>("/marker_detect/estimated",100);				// Publisher for Image of estimation
    pub_marker_id = nh.advertise < std_msgs::String > ("/marker_detect/id", 100);						// Publisher for ID of detected marker
    pub_pose = nh.advertise < geometry_msgs::PoseStamped > ("marker/pose", 1000);						// Publisher for pose of detected marker
    pub_velocity = nh.advertise < geometry_msgs::Twist > ("/cmd_vel", 1);							// Publisher for rover velocity
    viz_marker_pub = nh.advertise < visualization_msgs::Marker >("/tag_marker",100);					// Publisher for Visual Marker of Tag
    //sub_camera = nh.subscribe("/camera/color/image_raw", 100, & arucoDetection::imageCallback, this);			// Callback for Image from camera - Realsense d435
    sub_camera_sim = nh.subscribe("/zed2/zed_node/rgb/image_rect_color", 100, &arucoDetection::imageCallback,this);		// General Callback for testing
}

void arucoDetection::imageCallback(const sensor_msgs::ImageConstPtr & msg) {

    camera_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //std::cout<<"Image Recieved";
    arucoDetection::detect_aruco();
    //std::cout<<"Marker Detected";
    estimate_pose();
    //std::cout<<"Pose Estimated";
    msg_pub = camera_image -> toImageMsg();
    pub_marker.publish(msg_pub);
    
};

void arucoDetection::detect_aruco() {

    //set_params();
    
    markers_drawn_img = camera_image -> image;
    cv::aruco::detectMarkers(markers_drawn_img, dictionary, corners, marker_ids_detected, params);
    
    marker_ID.data = " ";
    
    for (int i = 0; i < marker_ids_detected.size(); i++) {
    
        ID = std::to_string(marker_ids_detected[i]);
        marker_ID.data.append(ID);
          
    }
    
    pub_marker_id.publish(marker_ID);
    
    if (marker_ids_detected.size() > 0) {
    
        cv::aruco::drawDetectedMarkers(markers_drawn_img, corners, marker_ids_detected);
    }
};

/*void arucoDetection::set_params() {

    params -> cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    params -> adaptiveThreshWinSizeMin = 5;
    params -> adaptiveThreshWinSizeMax = 20;
    params -> adaptiveThreshWinSizeStep = 5;
    params -> adaptiveThreshConstant = 25;
    params -> minMarkerPerimeterRate = 0.03;
    params -> maxMarkerPerimeterRate = 1;
    params -> polygonalApproxAccuracyRate = 0.052;
    params -> minCornerDistanceRate = 0.05;
    params -> minMarkerDistanceRate = 0.1;
    params -> minDistanceToBorder = 2;
    params -> cornerRefinementWinSize = 4;
    params -> cornerRefinementMinAccuracy = 0.1;
    params -> cornerRefinementMaxIterations = 50;
    //bsdk theory padh : params->minOtsuStdDev = 0.1;

};*/

void arucoDetection::estimate_pose() {
    final_image = camera_image -> image;
    
    // Caliberated Camera Matrix and distortionCoeffs for Zed 2
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at < double > (0, 0) =1059.59;
    cameraMatrix.at < double > (0, 2) = 1099.32;
    cameraMatrix.at < double > (1, 1) = 1059.29;
    cameraMatrix.at < double > (1, 2) = 601.431;
    
    distortionCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    distortionCoeffs.at < double > (0, 0) = -0.0430829;
    distortionCoeffs.at < double > (1, 0) = 0.011427;
    distortionCoeffs.at < double > (2, 0) = 4.65188e-06;
    distortionCoeffs.at < double > (3, 0) = -0.000150164;
    distortionCoeffs.at < double > (4, 0) = 0.00525169;

        if (marker_ids_detected.size() > 0) {
        
            cv::aruco::estimatePoseSingleMarkers(corners, 0.02, cameraMatrix, distortionCoeffs, RotationalVectors, TranslationalVectors);
            
            try {
            
                cv::aruco::drawAxis(final_image, cameraMatrix, distortionCoeffs, RotationalVectors[0], TranslationalVectors[0], 0.05);
            } 
            
            catch (...) {
                ROS_INFO_STREAM("\n Unable to draw axis.");
            }
            
            msg_pub_estimated = camera_image -> toImageMsg();
            pub_pose_estimated.publish(msg_pub_estimated);
            
            
            calculate_pose();
            

    }
};

void arucoDetection::calculate_pose() {

    for (int i = 0; i < TranslationalVectors.size(); ++i) {
        cv::Vec3d curr_RotVec;	
        cv::Vec3d curr_TransVec;
        curr_RotVec = RotationalVectors[i];
        curr_TransVec = TranslationalVectors[i];
        cv::Rodrigues(curr_RotVec, rotMat);
        if (cv::determinant(rotMat) > 0.99 && cv::determinant(rotMat) < 1.01) {
        
            origin = rotMat * -curr_TransVec;
            
            tf2::Quaternion q;
            q.setRPY(curr_RotVec[0], curr_RotVec[1], curr_RotVec[2]);
            q = q.normalize();
            
            geometry_msgs::Quaternion quaternion = tf2::toMsg(q);
            MarkerPose.pose.position.x = curr_TransVec[2];
            MarkerPose.pose.position.y = curr_TransVec[0];
            MarkerPose.pose.position.z = -curr_TransVec[1];
            MarkerPose.pose.orientation = quaternion;
            
            double dist = sqrt((curr_TransVec[0] * curr_TransVec[0]) + (curr_TransVec[1] * curr_TransVec[1]) + (curr_TransVec[2] * curr_TransVec[2]));
            ROS_INFO_STREAM("Marker Pose : \n");
            ROS_INFO_STREAM("x : " << curr_TransVec[2]);
            ROS_INFO_STREAM("y : " << -curr_TransVec[0]);
            ROS_INFO_STREAM("z : " << -curr_TransVec[1]);
            ROS_INFO_STREAM("Marker Distance : " << dist);
             visualization_msgs::Marker marker;
		    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
		    marker.header.frame_id = "zed2_base_link";
		    marker.header.stamp = ros::Time::now();

		    // Set the namespace and id for this marker.  This serves to create a unique ID
		    // Any marker sent with the same namespace and id will overwrite the old one
		    marker.ns = "basic_shapes";
		    marker.id = 0;

		    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		    marker.type = shape;

		    // Set the marker action.  Options are ADD, DELETE
		    marker.action = visualization_msgs::Marker::ADD;

		    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		    marker.pose.position.x = curr_TransVec[2]*10;
		    marker.pose.position.y = -curr_TransVec[0]*10;
		    marker.pose.position.z = -curr_TransVec[1]*10;
		    marker.pose.orientation = quaternion;

		    // Set the scale of the marker -- 1x1x1 here means 1m on a side
		    marker.scale.x = 1.0;
		    marker.scale.y = 0.1;
		    marker.scale.z = 1.0;

		    // Set the color -- be sure to set alpha to something non-zero!
		    marker.color.r = 0.0f;
		    marker.color.g = 1.0f;
		    marker.color.b = 0.0f;
		    marker.color.a = 1.0;
		    
		    marker.text = "Distance = "+std::to_string(dist);
		    //marker.lifetime = ros::Duration();
			
		    // Publish the marker
		    viz_marker_pub.publish(marker);
            
            if (marker_ids_detected.size() > 0) {
            
                times_detected = times_detected + 1;
            }
            
            if (times_detected > 3) {
                try {
                
                    nh.setParam("/marker_ids_detected", marker_ids_detected[0]);
                    nh.setParam("/marker/x", curr_TransVec[2]);
                    nh.setParam("/marker/y", curr_TransVec[0]);
                    nh.setParam("/marker/z", curr_TransVec[1]);
                    Vel.linear.x = 0;
                    Vel.angular.z = 0;
                    pub_velocity.publish(Vel);
                    //std::system("rosnode kill /search_pattern");
                    
                } 
                
                catch (...) {
                
                    ROS_INFO_STREAM("Search Pattern not running");
                }

            }
            pub_pose.publish(MarkerPose);
        }

    }
    
    /*
    Function for camera calliberation. Uncomment if needed only :
    
    void Aruco_Detection::create_checkboard()
   {
        std::cout << ""<<"\n";

        for (int i=0; i<boardSize.height; i++) {
            for (int j=0; j<boardSize.width; j++) {
                obj.push_back(Point3f(i, j, 0.0f));
            }
        }
   }*/
};

int main(int argc, char ** argv) {

    ros::init(argc, argv, "aruco_detection_node");
    arucoDetection object = arucoDetection();
    ros::spin();
}
