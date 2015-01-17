//  test_warping.cpp
//  STITCHTRON v9000

#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char ** argv) {

  std::string path1 = "/home/gareth/Documents/stitchtron9000/image/data2_1.jpg";
  std::string path2 = "/home/gareth/Documents/stitchtron9000/image/data2_2.jpg";
  
  const cv::Mat I1 = cv::imread(path1);
  const cv::Mat I2 = cv::imread(path2);

  cv::imshow("image1", I1);
  cv::imshow("image2", I2);
  
  cv::waitKey(0);
  
  return 0;
}
