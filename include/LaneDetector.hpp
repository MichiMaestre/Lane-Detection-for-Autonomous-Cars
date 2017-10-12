#pragma once
#include<iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>


class LaneDetector {
 private:
  cv::Mat img_noise;
  cv::Mat img_edges;

 public:

  cv::Mat deNoise(cv::Mat inputImage);
  cv::Mat edgeDetector(cv::Mat img_noise);
  cv::Mat mask(cv::Mat img_edges);
  std::vector<cv::Vec4i> houghLines(cv::Mat inputImage, cv::Mat img_mask);

};
