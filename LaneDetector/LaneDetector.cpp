#include "opencv2/opencv.hpp"
#include "../include/LaneDetector.hpp"


cv::Mat LaneDetector::deNoise(cv::Mat inputImage) {
  cv::Mat output;

  cv::GaussianBlur(inputImage, output, cv::Size(3,3), 0, 0);

  return output;
}

cv::Mat LaneDetector::edgeDetector(cv::Mat img_noise) {
  cv::Mat output;
  cv::Mat kernel;
  cv::Point anchor;


  cv::cvtColor(img_noise, output, cv::COLOR_RGB2GRAY);
  cv::threshold(output, output, 140, 255, cv::THRESH_BINARY);

  anchor = cv::Point( -1, -1 );
  kernel = cv::Mat(1, 3, CV_32F);
  kernel.at<float>(0,0) = -1;
  kernel.at<float>(0,1) = 0;
  kernel.at<float>(0,2) = 1;

  cv::filter2D(output, output, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);

  return output;
}
