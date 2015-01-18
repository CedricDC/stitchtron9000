//  mosaicer_node.hpp
//  STITCHTRON v9000

#include <ros/node_handle.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <stitchtron9000/Homography.h>

#include <memory>

#ifndef MOSAICER_NODE_HPP
#define MOSAICER_NODE_HPP

namespace s9000 {
namespace mosaicer {

class Node {
public:
  
  Node(const ros::NodeHandle& nh);
  
private:
  
  void callback(const sensor_msgs::ImageConstPtr&,
                const sensor_msgs::CameraInfoConstPtr&,
                const stitchtron9000::HomographyConstPtr&);
  
  static constexpr size_t kROSQueueSize = 100;
  
  ros::NodeHandle pnh_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  image_transport::SubscriberFilter sub_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
  message_filters::Subscriber<stitchtron9000::Homography> sub_homo_;
  
  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    sensor_msgs::CameraInfo,
    stitchtron9000::Homography> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  
  ros::Publisher pub_image_;
  
  int map_width_, map_height_;
  int offset_x_, offset_y_;
  cv::Mat mosaic_;
  cv::Mat homography_chain_;
};

} //  namespace mosaicer
} //  namespace s9000

#endif  //  MOSAICER_NODE_HPP
