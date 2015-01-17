#include <vector>
#include <iostream>
#include <ros/ros.h>
#include "stitchtron9000/Homography.h"
#include "stitchtron9000/KeyFrame.h"

// global variables
static stitchtron9000::Homography hom_prev;
static stitchtron9000::Homography hom_curr;

void keyframe_cb( const stitchtron9000::KeyFrame& msg ) {
	ROS_INFO("Reached keyframe_callback");
};


int main (int argc, char **argv) {

	// Initializing node
	ros::init(argc, argv, "homographer");
	ros::NodeHandle pnh;

	// Publisher
	ros::Publisher pub_homography = pnh.advertise<stitchtron9000::Homography>("homography",1);

	// Subscriber
	ros::Subscriber sub_keyframe = pnh.subscribe("key_frame",1,keyframe_cb);

	ros::spin();

	return 0;
};
