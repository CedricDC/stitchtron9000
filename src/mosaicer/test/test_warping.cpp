//  test_warping.cpp
//  STITCHTRON v9000

#include <iostream>
#include <opencv2/opencv.hpp>
#include <mosaicer/mosaicer_common.hpp>

using namespace s9000::mosaicer;

int main(int, char **) {

  std::string path1 = "/home/gareth/Documents/stitchtron9000/image/data2_1.jpg";
  std::string path2 = "/home/gareth/Documents/stitchtron9000/image/data2_2.jpg";
  
  const cv::Mat I1 = cv::imread(path1);
  const cv::Mat I2 = cv::imread(path2);

  const cv::Size small_size(I1.cols / 1, I2.rows / 1);
  
  cv::Mat I1_small, I2_small;
  cv::resize(I1, I1_small, small_size);
  cv::resize(I2, I2_small, small_size);
  
  cv::Mat map(2500,2500,I1_small.type());  
  cv::Mat H = cv::Mat::eye(3,3,CV_32F);
  H = (cv::Mat_<double>(3,3) << 1.0235,   -0.0110,  120.2061,
       0.0470,    0.9587,   70.8542,
       0.0001,   -0.0000,    1.0000);
  
  cv::Rect_<double> nrect;
  const cv::Mat Hnew = fitHomography(H,
                                     cv::Size(I2_small.cols,I2_small.rows),
                                     nrect);
  
  //  calculate pixel ROI
  cv::Rect roi;
  roi.x = std::ceil(nrect.x);
  roi.y = std::ceil(nrect.y);
  roi.width = std::floor(nrect.width);
  roi.height = std::floor(nrect.height);

  std::cout << nrect << std::endl;
  
  //  warp to the target region  
  cv::Mat map_slice(roi.size(), map.type());
  cv::Mat mask_slice(roi.size(), CV_8UC1);
  mask_slice.setTo(255);
  cv::warpPerspective(I2_small, map_slice, Hnew, roi.size());
  cv::warpPerspective(mask_slice, mask_slice, Hnew, roi.size());
  
  cv::imshow("map", map_slice);  
  cv::imshow("mask", mask_slice);
  
  cv::waitKey(0);
  
  return 0;
}
