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

cv::Mat LaneDetector::mask(cv::Mat img_edges) {
  cv::Mat output;
  cv::Mat mask = cv::Mat::zeros(img_edges.size(), img_edges.type());
  cv::Point pts[4] = {
      cv::Point(210, 720),
      cv::Point(550, 450),
      cv::Point(717, 450),
      cv::Point(1280, 720)
  };
  cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
  cv::bitwise_and(img_edges, mask, output);

  return output;
}

std::vector<cv::Vec4i> LaneDetector::houghLines(cv::Mat inputImage, cv::Mat img_mask) {
  std::vector<cv::Vec4i> line;

  HoughLinesP(img_mask, line, 1, CV_PI/180, 20, 20, 30);
  for( size_t i = 0; i < line.size(); i++ )
    {
      cv::Vec4i l = line[i];
      cv::line( inputImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
    }
  cv::namedWindow("Lines", CV_WINDOW_AUTOSIZE);
  cv::imshow("Lines", inputImage);

  return line;
}
