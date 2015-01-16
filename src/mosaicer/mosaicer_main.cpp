//  mosaicer_main.cpp
//  STITCHTRON v9000

#include <mosaicer/mosaicer_node.hpp>

int main(int argc, char ** argv) {
  ros::init(argc,argv,"mosaicer");
  ros::NodeHandle nh("~");
  s9000::mosaicer::Node(nh);
  ros::spin();
  return 0;
}
