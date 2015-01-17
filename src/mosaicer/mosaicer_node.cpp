//  mosaicer_node.cpp
//  STITCHTRON v9000

#include <mosaicer/mosaicer_node.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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
  
  pnh_.param("map_width", map_width_, 10000);
  pnh_.param("map_height", map_height_, 10000);
  
  /// @todo: add scale parameter...
  
  //  clamp dimensions to some reasonable minimum
  map_width_ = std::max(map_width_, 0);
  map_height_ = std::max(map_height_, 0);
  ROS_INFO("Initial map dimensions set to %i x %i", map_width_, map_height_);
}

void Node::callback(const sensor_msgs::ImageConstPtr& image,
                    const sensor_msgs::CameraInfoConstPtr& info, 
                    const stitchtron9000::HomographyConstPtr& homo) {
  
  cv_bridge::CvImageConstPtr bridged = cv_bridge::toCvShare(image);
  if (!bridged || bridged->image.empty()) {
    ROS_ERROR("Failed to get cv_bridge image");
    return;
  }
  const cv::Mat& input_image = bridged->image;
  
  cv::Mat H(3,3,CV_64F,0.0);
  if (homo) {
    //  convert homography to cv::Mat
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 3; j++) {
        H.at<double>(i,j) = homo->homography[i*3 + j];
      }
    }
  }
  
  if (mosaic_.empty()) {
    //  first image, allocate the map w/ same type as the input
    mosaic_ = cv::Mat(map_width_, map_height_, input_image.type());
    mosaic_.setTo(cv::Scalar(0)); //  black?
    
    /// @todo: initialize base homograpy
    
  } else {
    //  subsequent image, make sure type matches
    if (input_image.type() != mosaic_.type()) {
      ROS_ERROR("Uh oh! Input image does not have the same type. Ignoring it.");
      return; //  exit
    }
  }
  
  //  calculate homography w.r.t. first image (the map)
  const cv::Mat compounded_H = homography_chain_ * H;
  
  ROS_INFO_STREAM("Received homography : " << H);
  ROS_INFO_STREAM("Composite homography : " << compounded_H);
  
  //  warp onto mosaic
  cv::warpPerspective(input_image, mosaic_, compounded_H, 
                      cv::Size(mosaic_.cols, mosaic_.rows), 
                      cv::INTER_LINEAR);
  
  //  draw the updated image
  cv::imshow("mosaic", mosaic_);
  cv::waitKey(1);
  
  homography_chain_ = compounded_H;
}

} //  mosaicer
} //  s9000
