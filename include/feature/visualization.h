#ifndef STITCHTRON9000_VISUALIZATION_H_
#define STITCHTRON9000_VISUALIZATION_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace s9000 {
namespace feature {

void drawKeypoints(const std::vector<cv::KeyPoint> &keypoints, cv::Mat &image);

void drawKeypoints(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints,
                   cv::Mat &image_out);

void drawMatches(const cv::Mat &image1,
                 const std::vector<cv::KeyPoint> &keypoints1,
                 const cv::Mat &image2,
                 const std::vector<cv::KeyPoint> &keypoints2,
                 const std::vector<cv::DMatch> &matches, cv::Mat &image_match,
                 const std::vector<uchar> &matches_mask);

}  // namespace feature
}  // namespace s9000
#endif  // STITCHTRON9000_VISUALIZATION_H_
