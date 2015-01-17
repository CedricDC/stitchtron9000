#ifndef STITCHTRON9000_FEATURE_H_
#define STITCHTRON9000_FEATURE_H_

#include <ros/ros.h>
#include <opencv2/features2d/features2d.hpp>

namespace s9000 {
namespace feature {

class Feature2D {
 public:
  /**
   * @brief create
   * @param pnh
   * @param name
   * @return
   */
  static cv::Ptr<cv::Feature2D> create(const ros::NodeHandle& pnh,
                                       const std::string& name);
};

class FeatureDetector {
 public:
  /**
   * @brief create
   * @param pnh
   * @param name
   * @return
   */
  static cv::Ptr<cv::FeatureDetector> create(const ros::NodeHandle& pnh,
                                             const std::string& name);
};

/**
 * @brief SetCvAlgorithmParam
 * @param algorithm
 * @param fnh
 * @param param_name
 * @param default_value
 */
template <typename T>
void SetCvAlgorithmParam(cv::Algorithm* algo, const ros::NodeHandle& fnh,
                         const std::string& param_name, T default_value) {
  T value;
  fnh.param<T>(fnh.resolveName(param_name), value, default_value);
  algo->set(param_name, value);
}

void PrintCvAlgorithmParams(cv::Algorithm* algorithm);

}  // namespace featur
}  // namespace s9000

#endif  // STITCHTRON9000_FEATURE_H_
