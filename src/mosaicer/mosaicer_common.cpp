//  mosaicer_common.cpp
//  STITCHTRON v9000

#include <mosaicer/mosaicer_common.hpp>

namespace s9000 {
namespace mosaicer {

cv::Mat fitHomography(const cv::Mat& H,
                      const cv::Size src_size,
                      cv::Rect_<double> &dst_rect) {
  std::vector<cv::Mat> pts_0(4), pts_1(4);
   
  //  corners of the source image
  pts_0[0] = (cv::Mat_<double>(3,1) << 0, 0, 1);
  pts_0[1] = (cv::Mat_<double>(3,1) << 0, src_size.height, 1);
  pts_0[2] = (cv::Mat_<double>(3,1) << src_size.width, src_size.height, 1);
  pts_0[3] = (cv::Mat_<double>(3,1) << src_size.width, 0, 1);
  
  //  transform using perspective warp
  for (int i=0; i < 4; i++) {
    pts_1[i] = H * pts_0[i];
    pts_1[i] = pts_1[i] / pts_1[i].at<double>(2,0); //  divide by z
  }
  
  //  find the bounding rectangle
  double ox = std::numeric_limits<double>::infinity();
  double oy = ox;
  double sx = -ox;
  double sy = -ox;
  for (int i=0; i < 4; i++) {
    ox = std::min(ox, pts_1[i].at<double>(0,0));
    oy = std::min(oy, pts_1[i].at<double>(1,0));
    sx = std::max(sx, pts_1[i].at<double>(0,0));
    sy = std::max(sy, pts_1[i].at<double>(1,0));
  }
  //  determine size of rectangle in pixels
  sx -= ox;
  sy -= oy;
  
  //  subtract the origin
  for (int i=0; i < 4; i++) {
    pts_1[i].at<double>(0,0) -= ox;
    pts_1[i].at<double>(1,0) -= oy;
  }
  
  //  convert to cv::Point2f for cv::getPerspectiveTransform
  std::vector<cv::Point2f> ptsf_0, ptsf_1;
  for (int i=0; i < 4; i++) {
    ptsf_0.push_back(cv::Point2f(pts_0[i].at<double>(0,0),
                                 pts_0[i].at<double>(1,0)));
    ptsf_1.push_back(cv::Point2f(pts_1[i].at<double>(0,0),
                                 pts_1[i].at<double>(1,0)));
  }
  
  //  recompute a new homography
  cv::Mat Hnew = cv::getPerspectiveTransform(ptsf_0, ptsf_1);
 
  //  output the rect under the original homography
  dst_rect = cv::Rect_<double>(ox,oy,sx,sy);
  return Hnew;
}

}
}
