#include "extractor/extractor_node.h"

namespace s9000 {

ExtractorNode::ExtractorNode(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  pnh_.param("queue_size", queue_size_, 5);
  image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&ExtractorNode::ConnectCb, this);
  pub_key_frame_ = it_.advertise("", 1, connect_cb, connect_cb);
}

void ExtractorNode::ConnectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (pub_key_frame_.getNumSubscribers() == 0)
    sub_camera_.shutdown();
  else if (!sub_camera_) {
    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    sub_camera_ = it_.subscribeCamera("image", queue_size_,
                                      &ExtractorNode::CameraCb, this, hints);
    ROS_INFO("%s subscribing to %s.", pnh_.getNamespace().c_str(),
             sub_camera_.getTopic().c_str());
  }
}

void ExtractorNode::CameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  ROS_INFO("Inside CameraCb");
}

}  // namespace s9000
