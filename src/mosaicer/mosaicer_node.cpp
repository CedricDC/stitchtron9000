//  mosaicer_node.cpp
//  STITCHTRON v9000

#include <mosaicer/mosaicer_node.hpp>
#include <cv_bridge/cv_bridge.h>

using std::make_shared;

namespace s9000 {
namespace mosaicer {

Node::Node(const ros::NodeHandle &nh) : pnh_(nh) {
  
  it_.reset(new image_transport::ImageTransport(pnh_));
  
  sub_image_.subscribe(*it_, "image", kROSQueueSize);
  sub_info_.subscribe(pnh_, "camera_info", kROSQueueSize);
  sub_homo_.subscribe(pnh_, "homography", kROSQueueSize);
  
  sync_ = make_shared<message_filters::Synchronizer<SyncPolicy>>(
              SyncPolicy(kROSQueueSize), sub_image_, sub_info_, sub_homo_);
  sync_->registerCallback(boost::bind(&Node::callback,this, _1, _2, _3));
  
  pub_image_ = pnh_.advertise<sensor_msgs::Image>("mosaic_image", 1);
}

void Node::callback(const sensor_msgs::ImageConstPtr& image,
                    const sensor_msgs::CameraInfoConstPtr& info, 
                    const stitchtron9000::HomographyConstPtr& homo) {
  
  cv_bridge::CvImageConstPtr bridged = cv_bridge::toCvShare(image);
  if (!bridged || bridged->image.empty()) {
    ROS_ERROR("Failed to convert image to mono8");
    return;
  }
  
  ROS_INFO("Received image, info and homography!!");
  
  /// @todo: make a mosaic
}

} //  mosaicer
} //  s9000
