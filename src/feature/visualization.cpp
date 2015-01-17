#include "feature/visualization.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace s9000 {
namespace feature {

void drawCorners(const cv::Mat &image, const std::vector<cv::Point2f> &corners,
                 cv::Mat &image_out) {
  if (image.type() == CV_8UC3) {
    image.copyTo(image_out);
  } else if (image.type() == CV_8UC1) {
    cv::cvtColor(image, image_out, cv::COLOR_GRAY2BGR);
  } else {
    //    CV_Error(Error::StsBadArg, "Incorrect type of input image.\n");
  }
  for (const auto &p : corners) {
    cv::circle(image_out, p, 1, CV_BLUE, -1, CV_AA);
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

void drawMatches(const cv::Mat &image1,
                 const std::vector<cv::KeyPoint> &keypoints1,
                 const cv::Mat &image2,
                 const std::vector<cv::KeyPoint> &keypoints2,
                 const std::vector<cv::DMatch> &matches, cv::Mat &image_match,
                 const std::vector<uchar> &matches_mask) {
  // Such consistency in opencv where in drawMachages the type of mask is
  // std::vector<char> instead of std::vector<uchar> or cv::Mat
  std::vector<char> mask(matches_mask.cbegin(), matches_mask.cend());
  cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, image_match,
                  /* match color */ CV_RED, /* point color */ CV_GREEN, mask,
                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}
}  // namesapce feature
}  // namesapce s9000
