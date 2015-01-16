//  extractor_main.cpp
//  STITCHTRON v9000

#include <extractor/extractor_node.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "extractor");
  ros::NodeHandle pnh("~");
  s9000::extractor::ExtractorNode extractor(pnh);
  ros::spin();
}
