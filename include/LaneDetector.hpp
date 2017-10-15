#pragma once
#include<iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <tuple>


class LaneDetector {
 private:
  double img_size;
  double img_center;
  bool left_flag;
  bool right_flag;
  cv::Point right_b;
  double right_m;
  cv::Point left_b;
  double left_m;

 public:

  cv::Mat deNoise(cv::Mat inputImage);
  cv::Mat edgeDetector(cv::Mat img_noise);
  cv::Mat mask(cv::Mat img_edges);
  std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);
  std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges);
  std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage);
  std::string predictTurn();
  void plotLane(cv::Mat inputImage, std::vector<cv::Point>, std::string turn);
};
