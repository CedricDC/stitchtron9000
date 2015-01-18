#include "homographer/homographer_node.h"
#include <iostream>

namespace s9000 {
namespace homographer {

HomographerNode::HomographerNode(const ros::NodeHandle& nh) : pnh_(nh) {

	// subscribe to keyframe stream
	sub_keyframe_ = pnh_.subscribe("key_frame", 5, &HomographerNode::keyframe_cb, this);

	// publish homography
	pub_homography_ = pnh_.advertise<stitchtron9000::Homography>("homography",5);

}

// Callback for KeyFrame
void HomographerNode::keyframe_cb(const stitchtron9000::KeyFrame& msg) {
  ROS_INFO("New keyframe received");
  hom_curr_.header = msg.header;

  std::vector<cv::Point2f> points_curr;  // for coordinates of current keyframe
  std::vector<int> id_curr;              // for indices of current keyframe
  std::vector<cv::Point2f> tracked_pts_prev;
  std::vector<cv::Point2f> tracked_pts_curr;

  // 1) copy all features
  for (const auto& fet : msg.features) {
    points_curr.push_back(
        cv::Point2f(fet.x, fet.y));  // constructing feature vector
    id_curr.push_back(fet.id);       // constructing image index vector
  }

  if (id_prev_.empty()) {
    ROS_INFO("First image received");

    // Return identity matrix
    hom_curr_.homography[0] = hom_curr_.homography[4] = hom_curr_.homography[8] =
        1;

    std::cout << "Initial homography matrix: " << std::endl;
    this->print_homography();
  } else {
    ROS_INFO("New points copied");

    // 2) now fill two new vectors with tracked points
    //    since we constructed the tracked vector as intersection,
    //	  the element will certainly be present (not rechecking)
    for (const auto& fet : msg.tracked_ids) {
      // find in previous points
      int pos_prev =
          std::find(id_prev_.begin(), id_prev_.end(), fet) - id_prev_.begin();
      tracked_pts_prev.push_back(points_prev_[pos_prev]);

      // find in current (new) points
      int pos_curr =
          std::find(id_curr.begin(), id_curr.end(), fet) - id_curr.begin();
      tracked_pts_curr.push_back(points_curr[pos_curr]);
    }
    ROS_ASSERT_MSG(tracked_pts_curr.size() == tracked_pts_prev.size(),
                   "points size mismatch");
    ROS_INFO("Common elements extracted");

    // 3) extract homography matrix
    cv::Mat homography_mat =
        cv::findHomography(tracked_pts_curr, tracked_pts_prev, CV_RANSAC);

    // copy matrix into c++ container and publish
    for (int i = 0; i < 3; ++i) {
      const auto* p = homography_mat.ptr<double>(i);
      for (int j = 0; j < 3; ++j) {
        hom_curr_.homography[j + 3 * i] = p[j];
      }
    }

    std::cout << "New homography matrix: " << std::endl;
    this->print_homography();
  }

  // publish homography and update previous points
  pub_homography_.publish(hom_curr_);
  points_prev_ = points_curr;
  id_prev_ = id_curr;
}

// Printing homography matrix
void HomographerNode::print_homography() const {
  for (int i = 0; i < 3; ++i) {
    std::cout << this->hom_curr_.homography[0 + 3 * i] << ","
              << this->hom_curr_.homography[1 + 3 * i] << ","
              << this->hom_curr_.homography[2 + 3 * i] << "," << std::endl;
  }
}

} // end namespace homographer
} // end namespace s9000
