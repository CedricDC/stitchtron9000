//  mosaicer_common.hpp
//  STITCHTRON v9000

#ifndef MOSAICER_COMMON_HPP
#define MOSAICER_COMMON_HPP

#include <opencv2/opencv.hpp>

namespace s9000 {
namespace mosaicer {

cv::Mat fitHomography(const cv::Mat& H,
                      const cv::Size src_size,
                      cv::Rect_<double> &dst_rect);

}
}

#endif // MOSAICER_COMMON_HPP
