#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main() {
  cv::Mat mask = cv::Mat::zeros(500, 500, CV_8UC1);
  cv::circle(mask, cv::Point2f(250, 250), 50, cv::Scalar(255), -1);
  cv::imshow("mask", mask);
  cv::waitKey(-1);
}
