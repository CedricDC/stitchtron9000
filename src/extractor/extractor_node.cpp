#include "extractor/extractor_node.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stitchtron9000/KeyFrame.h>
#include <stitchtron9000/Homography.h>
#include <feature/feature2d.h>
#include <feature/visualization.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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
  //  feat2d_ = feature::Feature2D::create(pnh, algo_name);
  //  ROS_INFO("Feature2D %s", algo_name.c_str());
  //  feature::printCvAlgorithmParams(feat2d_);
  detector_ = cv::FeatureDetector::create("GFTT");
  detector_->set("minDistance", 20);
  feature::printCvAlgorithmParams(detector_);

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
      cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
  cv::Mat display;

  ///@todo: change all these magic numbers later
  if (!prev_image_.empty()) {
    ROS_INFO_THROTTLE(4, "Tracking features");
    // Not the first image, so we do tracking here
    ROS_ASSERT_MSG(!prev_features_.empty(), "No tracked features");

    // optical flow
    std::vector<cv::Point2f> prev_points, points;
    for (const stitchtron9000::Feature& feat : prev_features_) {
      prev_points.emplace_back(feat.x, feat.y);
    }
    std::vector<uchar> status;
    const cv::TermCriteria term_criteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.005);
    cv::calcOpticalFlowPyrLK(prev_image_, image, prev_points, points, status,
                             cv::noArray(), cv::Size(13, 13), 3, term_criteria);
    feature::pruneByStatus(status, prev_points);
    feature::pruneByStatus(status, points);
    feature::pruneByStatus(status, prev_features_);
    //    ROS_ASSERT_MSG(prev_points.size() == points.size(),
    //                   "Previous and current points mismatch");
    //    ROS_ASSERT_MSG(!points.empty(), "No points being tracked");
    //    ROS_INFO("Number of tracked points after optical flow: %d",
    //             (int)points.size());

    // Visualization
    //    feature::drawTrackes(prev_image_, prev_points, image, points,
    //                         std::vector<uchar>(), display);

    // Outlier rejection
    status.clear();
    cv::findFundamentalMat(prev_points, points, status, cv::FM_RANSAC, 1, 0.99);
    feature::pruneByStatus(status, prev_points);
    feature::pruneByStatus(status, points);
    feature::pruneByStatus(status, prev_features_);
    //    ROS_ASSERT_MSG(prev_points.size() == points.size(),
    //                   "Previous and current points mismatch");
    //    ROS_ASSERT_MSG(!points.empty(), "No points being tracked");
    //    ROS_INFO("Number of tracked points after outlier rejection: %d",
    //             (int)points.size());

    // Visualization
    feature::drawTrackes(prev_image_, prev_points, image, points,
                         std::vector<uchar>(), display);

    // points become prev_features
    std::vector<stitchtron9000::Feature> curr_features;
    for (size_t i = 0; i < points.size(); ++i) {
      stitchtron9000::Feature feature;
      feature.id = prev_features_[i].id;
      feature.x = points[i].x;
      feature.y = points[i].y;
      curr_features.push_back(feature);
    }
    prev_features_ = curr_features;
    //    ROS_INFO("Number of tracked features: %d",
    //    (int)prev_features_.size());
  }

  if (prev_features_.size() < 600) {
    // Too few features, this is a key frame and we detect more features
    ROS_WARN("Adding a new keyframe");
    stitchtron9000::KeyFrame key_frame;
    key_frame.header = image_msg->header;

    // Create a mask based on these features
    // Extract tracked_ids from tracked features
    cv::Mat mask = cv::Mat::ones(image_msg->height, image_msg->width, CV_8UC1);
    for (const stitchtron9000::Feature& feat : prev_features_) {
      key_frame.tracked_ids.push_back(feat.id);
      cv::circle(mask, cv::Point2f(feat.x, feat.y), 20, cv::Scalar(0), -1);
    }
    ROS_INFO("Number of tracked ids: %d", (int)key_frame.tracked_ids.size());

    // Detect new keypoints
    cv::Mat image_gray = feature::copyToGray(image);
    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(image_gray, keypoints, mask);

    //    if (display.empty()) display = image;
    feature::drawKeypoints(image, keypoints, display);

    // Add newly detected keypoints to prev_features
    // Now, prev_features are in fact tracked curr_features
    for (const cv::KeyPoint& keypoint : keypoints) {
      stitchtron9000::Feature feature;
      feature.id = feature_id_++;
      feature.x = keypoint.pt.x;
      feature.y = keypoint.pt.y;
      prev_features_.push_back(feature);
    }
    /// hack
    ROS_INFO("Number of features in key frame: %d", (int)prev_features_.size());
    key_frame.features = prev_features_;
    pub_key_frame_.publish(key_frame);
    // Save image to disk
//    const std::string file_name("/home/gareth/Desktop/" +
//                                std::to_string(image_id_++) + ".png");
//    cv::imwrite(file_name, image);
//    ROS_INFO("Image saved to %s", file_name.c_str());
  }

  prev_image_ = image;

  // Visualization
  // cv::imshow("display", display);
  // cv::waitKey(1);
}

}  // namespace extractor
}  // namespace s9000
