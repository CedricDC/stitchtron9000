#include "feature/feature2d.h"

#include <opencv2/nonfree/nonfree.hpp>

namespace s9000 {
namespace feature {

cv::Ptr<cv::Feature2D> Feature2D::create(const ros::NodeHandle &pnh,
                                         const std::string &name) {
  // NodeHandle of Feature2D
  ros::NodeHandle algo_nh(pnh, name);
  auto algo = cv::Feature2D::create(name);

  if (name == "BRISK") {
    SetCvAlgorithmParam(algo, algo_nh, "thres", 4);
    SetCvAlgorithmParam(algo, algo_nh, "octaves", 4);
    //  } else if (name == "SIFT") {
    //    cv::initModule_nonfree();
    //    SetCvAlgorithmParam(algo, algo_nh, "nFeatures", 0);
    //    SetCvAlgorithmParam(algo, algo_nh, "contrastThreshold", 0.04);
    //    SetCvAlgorithmParam(algo, algo_nh, "edgeThreshold", 10.0);
    //  } else if (name == "SURF") {
    //    cv::initModule_nonfree();
    //    SetCvAlgorithmParam(algo, algo_nh, "hessianThreshold", 500.0);
    //    SetCvAlgorithmParam(algo, algo_nh, "nOctaves", 4);
    //    SetCvAlgorithmParam(algo, algo_nh, "nOctaveLayers", 3);
    //  } else if (name == "AKAZE") {
    //    SetCvAlgorithmParam(algo, algo_nh, "nFeatures", 0);
    //    SetCvAlgorithmParam(algo, algo_nh, "oMax", 4);
    //    SetCvAlgorithmParam(algo, algo_nh, "nSubLevels", 4);
  } else {
    return nullptr;
  }

  return algo;
}

cv::Ptr<cv::FeatureDetector> FeatureDetector::create(const ros::NodeHandle &pnh,
                                                     const std::string &name) {
  // NodeHandle of FeatureDetector
  ros::NodeHandle algo_nh(pnh, name);
  auto algo = cv::FeatureDetector::create(name);

  return nullptr;
}

void PrintCvAlgorithmParams(cv::Algorithm *algorithm) {
  std::vector<std::string> params;
  algorithm->getParams(params);

  for (const std::string &param_name : params) {
    const auto type = algorithm->paramType(param_name);
    const auto help_text = algorithm->paramHelp(param_name);
    std::string type_text;

    switch (type) {
      case cv::Param::BOOLEAN:
        type_text = "bool";
        break;
      case cv::Param::INT:
        type_text = "int";
        break;
      case cv::Param::REAL:
        type_text = "real (double)";
        break;
      case cv::Param::STRING:
        type_text = "string";
        break;
      case cv::Param::MAT:
        type_text = "Mat";
        break;
      case cv::Param::ALGORITHM:
        type_text = "Algorithm";
        break;
      case cv::Param::MAT_VECTOR:
        type_text = "Mat vector";
        break;
    }
    std::cout << "Parameter name: " << param_name << " type: " << type_text
              << std::endl;
  }
}

}  // namespace feature
}  // namespace s9000
