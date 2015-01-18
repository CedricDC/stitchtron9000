#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "stitchtron9000/Homography.h"
#include "stitchtron9000/KeyFrame.h"

// global variables
static stitchtron9000::Homography
    hom_curr;  // to store current homography matrix
static std::vector<cv::Point2f>
    points_prev;                  // for coordinates of current keyframe
static std::vector<int> id_prev;  // for indices of current keyframe

static ros::Subscriber sub_keyframe;
static ros::Publisher pub_homography;

void print_homography(const stitchtron9000::Homography& homography);

// Callback for KeyFrame
void keyframe_cb(const stitchtron9000::KeyFrame& msg) {
  ROS_INFO("New keyframe received");
  hom_curr.header = msg.header;

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

  if (id_prev.empty()) {
    ROS_INFO("First image received");

    // Return identity matrix
    hom_curr.homography[0] = hom_curr.homography[4] = hom_curr.homography[8] =
        1;

    std::cout << "Initial homography matrix: " << std::endl;
    print_homography(hom_curr);
  } else {
    ROS_INFO("New points copied");

    // 2) now fill two new vectors with tracked points
    //    since we constructed the tracked vector as intersection,
    //	  the element will certainly be present (not rechecking)
    for (const auto& fet : msg.tracked_ids) {
      // find in previous points
      int pos_prev =
          std::find(id_prev.begin(), id_prev.end(), fet) - id_prev.begin();
      tracked_pts_prev.push_back(points_prev[pos_prev]);

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
        hom_curr.homography[j + 3 * i] = p[j];
      }
    }

    std::cout << "New homography matrix: " << std::endl;
    print_homography(hom_curr);
  }

  // publish homography and update previous points
  pub_homography.publish(hom_curr);
  points_prev = points_curr;
  id_prev = id_curr;
}

void print_homography(const stitchtron9000::Homography& homography) {
  for (int i = 0; i < 3; ++i) {
    std::cout << homography.homography[0 + 3 * i] << ","
              << homography.homography[1 + 3 * i] << ","
              << homography.homography[2 + 3 * i] << "," << std::endl;
  }
}

int main(int argc, char** argv) {
  // Initializing node
  ros::init(argc, argv, "homographer");
  ros::NodeHandle pnh;

  // Publisher
  pub_homography =
      pnh.advertise<stitchtron9000::Homography>("homography", 5);

  // Subscriber
  ros::Subscriber sub_keyframe = pnh.subscribe("key_frame", 5, keyframe_cb);

  ros::spin();

  return 0;
};
