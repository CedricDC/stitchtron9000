#ifndef STITCHTRON9000_FEATURE_H_
#define STITCHTRON9000_FEATURE_H_

#include <ros/ros.h>
#include <opencv2/features2d/features2d.hpp>

namespace s9000 {
namespace feature {

class Feature2D {
 public:
  static cv::Ptr<cv::Feature2D> create(const ros::NodeHandle& pnh,
                                       const std::string& name);
};

class FeatureDetector {
 public:
  static cv::Ptr<cv::FeatureDetector> create(const ros::NodeHandle& pnh,
                                             const std::string& name);
};

class CornerDetector {};

template <typename T>
void setCvAlgorithmParam(cv::Algorithm* algo, const ros::NodeHandle& fnh,
                         const std::string& param_name, T default_value) {
  T value;
  fnh.param<T>(fnh.resolveName(param_name), value, default_value);
  algo->set(param_name, value);
}

void printCvAlgorithmParams(cv::Algorithm* algo);

template <typename T, typename U>
void pruneByStatus(const std::vector<U>& status, std::vector<T>& objects) {
  ROS_ASSERT_MSG(status.size() == objects.size(),
                 "status and object size mismatch");
  ROS_ASSERT_MSG(!status.empty(), "nothing to prune");
  auto it_obj = objects.begin();
  for (const auto& s : status) {
    if (s) {
      it_obj++;
    } else {
      it_obj = objects.erase(it_obj);
    }
  }
}

}  // namespace featur
}  // namespace s9000

#endif  // STITCHTRON9000_FEATURE_H_
