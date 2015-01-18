#include "homographer/homographer_node.h"

int main(int argc, char** argv) {
	// Initializing node
	ros::init(argc, argv, "homographer");
	ros::NodeHandle pnh("~");
	s9000::homographer::HomographerNode homographer(pnh);
	ros::spin();

	return 0;
}
