#ifndef STITCHTRON9000_EXTRACTOR_NODE_H_
#define STITCHTRON9000_EXTRACTOR_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <mutex>

namespace s9000 {

class ExtractorNode {
 public:
  explicit ExtractorNode(const ros::NodeHandle& pnh);

 private:
  void CameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& cinfo_msg);
  void ConnectCb();

  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  ros::Publisher pub_key_frame_;
  std::mutex connect_mutex_;
  int queue_size_;
};

}  // namespace s9000

#endif  // STITCHTRON9000_EXTRACTOR_NODE_H_
