#pragma once
#include<iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>


class LaneDetector {
 private:
  cv::Mat img_noise;

 public:

  cv::Mat deNoise(cv::Mat inputImage);
  cv::Mat edgeDetector(cv::Mat img_noise);

};
