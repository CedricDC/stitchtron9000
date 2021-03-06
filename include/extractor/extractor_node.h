#ifndef STITCHTRON9000_EXTRACTOR_NODE_H_
#define STITCHTRON9000_EXTRACTOR_NODE_H_

#include <mutex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stitchtron9000/Feature.h>

#include "feature/feature2d.h"

namespace s9000 {
namespace extractor {

class ExtractorNode {
 public:
  explicit ExtractorNode(const ros::NodeHandle& pnh);

 private:
  void cameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& cinfo_msg);
  void connectCb();

  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  ros::Publisher pub_key_frame_;
  std::mutex connect_mutex_;
  cv::Ptr<cv::FeatureDetector> detector_;
  std::vector<stitchtron9000::Feature> prev_features_;
  cv::Mat prev_image_;
  int feature_id_ = 0;
  int queue_size_;
  int image_id_ = 0;
};

}  // namespace extractor
}  // namespace s9000

#endif  // STITCHTRON9000_EXTRACTOR_NODE_H_
