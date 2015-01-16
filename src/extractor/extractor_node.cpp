#include "extractor/extractor_node.h"

#include <cv_bridge/cv_bridge.h>
#include <stitchtron9000/KeyFrame.h>
#include <stitchtron9000/Homography.h>

#include <opencv2/highgui/highgui.hpp>

namespace s9000 {
namespace extractor {

ExtractorNode::ExtractorNode(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  pnh_.param("queue_size", queue_size_, 5);
  image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
  sub_camera_ = it_.subscribeCamera("image", queue_size_,
                                    &ExtractorNode::CameraCb, this, hints);
  ROS_INFO("%s subscribing to %s.", pnh_.getNamespace().c_str(),
           sub_camera_.getTopic().c_str());
  pub_key_frame_ = pnh_.advertise<stitchtron9000::KeyFrame>("key_frame", 1);
  ROS_INFO("%s publishing to %s.", pnh_.getNamespace().c_str(),
           pub_key_frame_.getTopic().c_str());
  //  ros::SubscriberStatusCallback connect_cb =
  //      boost::bind(&ExtractorNode::ConnectCb, this);
  //  pub_key_frame_ = pnh_.advertise<stitchtron9000::KeyFrame>(
  //      "key_frame", 1, connect_cb, connect_cb);
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
  const cv::Mat image =
      cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
  cv::imshow("image", image);
  cv::waitKey(1);
  stitchtron9000::KeyFrame key_frame;
  key_frame.header = image_msg->header;
  pub_key_frame_.publish(key_frame);
}

}  // namespace extractor
}  // namespace s9000
