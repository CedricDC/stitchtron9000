#include <iostream>
#include "feature/visualization.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace s9000 {
namespace feature {

cv::Mat copyToGray(const cv::Mat &image) {
  cv::Mat image_gray;
  if (image.type() == CV_8UC3) {
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  } else if (image.type() == CV_8UC1) {
    image.copyTo(image_gray);
  } else {
    throw std::runtime_error("Incorrect type of input image.");
  }
  return image_gray;
}

cv::Mat copyToColor(const cv::Mat &image) {
  cv::Mat image_color;
  if (image.type() == CV_8UC3) {
    image.copyTo(image_color);
  } else if (image.type() == CV_8UC1) {
    cv::cvtColor(image, image_color, cv::COLOR_GRAY2BGR);
  } else {
    //    CV_Error(Error::StsBadArg, "Incorrect type of input image.\n");
    throw std::runtime_error("Incorrect type of input image.");
  }
  return image_color;
}

void drawCorners(const cv::Mat &image, const std::vector<cv::Point2f> &corners,
                 cv::Mat &image_out) {
  image_out = copyToColor(image);
  for (const auto &corner : corners) {
    cv::circle(image_out, corner, 1, CV_BLUE, -1, CV_AA);
  }
}

void drawKeypoints(const std::vector<cv::KeyPoint> &keypoints, cv::Mat &image) {
  for (const cv::KeyPoint &keypoint : keypoints) {
    cv::circle(image, keypoint.pt, 1, CV_BLUE, -1, CV_AA);  // center
    cv::circle(image, keypoint.pt, keypoint.size, CV_GREEN, 1, CV_AA);
  }
}

void drawKeypoints(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints,
                   cv::Mat &image_out) {
  cv::drawKeypoints(image, keypoints, image_out, CV_GREEN,
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

void drawTrackes(const cv::Mat &image1,
                 const std::vector<cv::Point2f> &corners1,
                 const cv::Mat &image2,
                 const std::vector<cv::Point2f> &corners2,
                 const std::vector<uchar> &matches, cv::Mat &image_track) {
  double alpha = 0.2;
  double beta = (1.0 - alpha);
  cv::addWeighted(image1, alpha, image2, beta, 0.0, image_track);
  // Process matches
  std::vector<uchar> matches_;
  if (matches.empty()) {
    matches_ = std::vector<uchar>(corners2.size(), 1);
  } else {
    matches_ = matches;
  }
  for (size_t i = 0; i < matches_.size(); ++i) {
    cv::circle(image_track, corners2[i], 3, CV_BLUE, -1, CV_AA);
    cv::line(image_track, corners1[i], corners2[i], CV_RED, 2);
  }
}

void drawMatches(const cv::Mat &image1,
                 const std::vector<cv::KeyPoint> &keypoints1,
                 const cv::Mat &image2,
                 const std::vector<cv::KeyPoint> &keypoints2,
                 const std::vector<cv::DMatch> &matches, cv::Mat &image_match,
                 const std::vector<uchar> &matches_mask) {
  // Such consistency in opencv where in drawMatches the type of mask is
  // std::vector<char> instead of std::vector<uchar> or cv::Mat
  std::vector<char> mask(matches_mask.cbegin(), matches_mask.cend());
  cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, image_match,
                  /* match color */ CV_RED, /* point color */ CV_GREEN, mask,
                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}
}  // namesapce feature
}  // namesapce s9000
