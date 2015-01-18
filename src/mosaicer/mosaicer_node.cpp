//  mosaicer_node.cpp
//  STITCHTRON v9000

#include <mosaicer/mosaicer_node.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <mosaicer/mosaicer_common.hpp>

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
  
  pnh_.param("map_width", map_width_, 5000);
  pnh_.param("map_height", map_height_, 5000);
  
  /// @todo: add scale parameter...
  
  //  clamp dimensions to some reasonable minimum
  map_width_ = std::max(map_width_, 0);
  map_height_ = std::max(map_height_, 0);
  ROS_INFO("Initial map dimensions set to %i x %i", map_width_, map_height_);
  
  cv::namedWindow("mosaic", cv::WINDOW_NORMAL);
  cv::namedWindow("blah", cv::WINDOW_NORMAL);
}

void Node::callback(const sensor_msgs::ImageConstPtr& image,
                    const sensor_msgs::CameraInfoConstPtr& info, 
                    const stitchtron9000::HomographyConstPtr& homo) {

  ROS_INFO("callback!");
  cv_bridge::CvImageConstPtr bridged = cv_bridge::toCvCopy(image, "rgb8");
  if (!bridged || bridged->image.empty()) {
    ROS_ERROR("Failed to get cv_bridge image");
    return;
  }
  const cv::Mat input_image = bridged->image;
  
//  cv::Mat H(3,3,CV_64F,0.0);
//  if (homo) {
//    //  convert homography to cv::Mat
//    for (int i=0; i < 3; i++) {
//      for (int j=0; j < 3; j++) {
//        H.at<double>(i,j) = homo->homography[i*3 + j];
//      }
//    }
//  }
  
//  if (mosaic_.empty()) {
//    ROS_INFO("Initializing mosaic");
    
//    //  first image, allocate the map w/ same type as the input
//    mosaic_ = cv::Mat(map_height_, map_width_, 
//                      input_image.type(),cv::Scalar(0));
    
//    offset_x_ = map_width_ / 2;
//    offset_y_ = map_height_ / 2;
    
//    //   initialize the base homography
//    homography_chain_ = cv::Mat::eye(3,3,CV_64F);
//  } else {
//    //  subsequent image, make sure type matches
//    if (input_image.type() != mosaic_.type()) {
//      ROS_ERROR("Uh oh! Input image does not have the same type. Ignoring it.");
//      return; //  exit
//    }
//  }
  
//  //  calculate homography w.r.t. first image (the map)
//  const cv::Mat compounded_H = homography_chain_ * H;
  
//  ROS_INFO_STREAM("Received homography : " << H);
//  ROS_INFO_STREAM("Composite homography : " << compounded_H);
  
//  //  adjust our homography
//  const cv::Size src_size(input_image.cols, input_image.rows);
  
//  cv::Rect_<double> dst_rect_f;
//  const cv::Mat adjusted_H = fitHomography(compounded_H,
//                                           src_size, dst_rect_f, 1.0);
//  cv::Rect dst_rect = dst_rect_f; //  rounded bounding box
  
//  //  shift to origin
//  dst_rect.x += offset_x_;
//  dst_rect.y += offset_y_;
  
//  ROS_INFO_STREAM("Adjusted homography : " << adjusted_H);
  
//  ROS_INFO_STREAM("Target region : " << dst_rect);
  
//  //  warp both the input and a white mask
//  cv::Mat input_warped(dst_rect.size(), mosaic_.type());
//  cv::Mat region_mask(src_size, CV_8UC1);
//  region_mask.setTo(255);
  
//  cv::warpPerspective(input_image, input_warped, adjusted_H, 
//                      dst_rect.size(), cv::INTER_LINEAR);
//  cv::warpPerspective(region_mask, region_mask, adjusted_H,
//                      dst_rect.size(), cv::INTER_LINEAR);
  
//  //  copy using the mask into the ROI
//  /// @todo: add some blending...
//  cv::Mat target = mosaic_(dst_rect);
//  input_warped.copyTo(target, region_mask);
  
  //cv::Mat small;
  //cv::resize(mosaic_,small,cv::Size(mosaic_.cols / 5, mosaic_.rows / 5));
  //cv::imshow("mosaic", mosaic_);
  
  cv::imshow("blah", input_image);
  
  
  cv::waitKey(1);
  
  
  //homography_chain_ = compounded_H;
}

} //  mosaicer
} //  s9000
