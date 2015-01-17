#include "extractor/extractor_node.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stitchtron9000/KeyFrame.h>
#include <stitchtron9000/Homography.h>
#include <feature/feature2d.h>
#include <feature/visualization.h>

#include <opencv2/highgui/highgui.hpp>

namespace s9000 {
namespace extractor {

ExtractorNode::ExtractorNode(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  pnh_.param("queue_size", queue_size_, 5);
  image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
  sub_camera_ = it_.subscribeCamera("image", queue_size_,
                                    &ExtractorNode::cameraCb, this, hints);
  ROS_INFO("%s subscribing to %s.", pnh_.getNamespace().c_str(),
           sub_camera_.getTopic().c_str());
  pub_key_frame_ = pnh_.advertise<stitchtron9000::KeyFrame>("key_frame", 1);
  ROS_INFO("%s publishing to %s.", pnh_.getNamespace().c_str(),
           pub_key_frame_.getTopic().c_str());
  std::string algo_name;
  if (!pnh_.getParam("Feature2D", algo_name)) {
    throw std::runtime_error("No Feature2D specified.");
  }
  feat2d_ = feature::Feature2D::create(pnh, algo_name);
  ROS_INFO("Feature2D %s", algo_name.c_str());
  feature::printCvAlgorithmParams(feat2d_);

  //  ros::SubscriberStatusCallback connect_cb =
  //      boost::bind(&ExtractorNode::ConnectCb, this);
  //  pub_key_frame_ = pnh_.advertise<stitchtron9000::KeyFrame>(
  //      "key_frame", 1, connect_cb, connect_cb);
}

void ExtractorNode::connectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (pub_key_frame_.getNumSubscribers() == 0)
    sub_camera_.shutdown();
  else if (!sub_camera_) {
    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    sub_camera_ = it_.subscribeCamera("image", queue_size_,
                                      &ExtractorNode::cameraCb, this, hints);
    ROS_INFO("%s subscribing to %s.", pnh_.getNamespace().c_str(),
             sub_camera_.getTopic().c_str());
  }
}

void ExtractorNode::cameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  const cv::Mat image =
      cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8)
          ->image;
  cv::Mat display;
  std::vector<cv::KeyPoint> keypoints;
  feat2d_->detect(image, keypoints);
  feature::drawKeypoints(image, keypoints, display);
  cv::imshow("display", display);
  cv::waitKey(1);

  // Publish dummy message
  stitchtron9000::KeyFrame key_frame;
  key_frame.header = image_msg->header;
  pub_key_frame_.publish(key_frame);
}

}  // namespace extractor
}  // namespace s9000
