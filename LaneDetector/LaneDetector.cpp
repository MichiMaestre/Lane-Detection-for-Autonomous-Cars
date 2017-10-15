#include "opencv2/opencv.hpp"
#include "../include/LaneDetector.hpp"
#include <math.h>

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

std::vector<cv::Vec4i> LaneDetector::houghLines(cv::Mat img_mask) {
  std::vector<cv::Vec4i> line;

  HoughLinesP(img_mask, line, 1, CV_PI/180, 20, 20, 30);
//  for( size_t i = 0; i < line.size(); i++ )
//    {
//      cv::Vec4i l = line[i];
//      cv::line( inputImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
//    }
//  cv::namedWindow("Lines", CV_WINDOW_AUTOSIZE);
//  cv::imshow("Lines", inputImage);
//  std::cout << line.size() << std::endl;

  return line;
}

std::vector<std::vector<cv::Vec4i> > LaneDetector::lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
  std::vector<std::vector<cv::Vec4i> > output(2);
  cv::Point ini;
  cv::Point fini;
  double slope;
  double slope_thresh = 0.3;
  std::vector<double> slopes;
  std::vector<cv::Vec4i> selected_lines;

  // Get the slope of all the detected lines
  for(auto i : lines) {
    ini = cv::Point(i[0], i[1]);
    fini = cv::Point(i[2], i[3]);

    if ((double)fini.x - (double)ini.x == 0)
        slope = 999;
    else
      slope = ((double)fini.y - (double)ini.y)/((double)fini.x - (double)ini.x);

    if (std::abs(slope) > slope_thresh) {
      slopes.push_back(slope);
      selected_lines.push_back(i);
    }
  }

  // Split the lines into right and left lines
  img_center = (double)(img_edges.cols / 2);
  std::vector<cv::Vec4i> right_lines, left_lines;
//  bool right_flag, left_flag;

  for (size_t j = 0; j < selected_lines.size(); j++) {
    ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
    fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);

    if (slopes[j] > 0 && fini.x > img_center && ini.x > img_center) {
      right_lines.push_back(selected_lines[j]);
      right_flag = true;
    }
    else if (slopes[j] < 0 && fini.x < img_center && ini.x < img_center) {
      left_lines.push_back(selected_lines[j]);
      left_flag = true;
    }
    else {
      right_flag = false;
      left_flag = false;
      std::cout << ini.x << std::endl;
    }
  }

  output[0] = right_lines;
  output[1] = left_lines;

  return output;
}

std::vector<cv::Point> LaneDetector::regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage) {
  std::vector<cv::Point> output(4);
  cv::Point ini;
  cv::Point fini;
  cv::Point ini2;
  cv::Point fini2;
  std::vector<cv::Point> right_pts;
  cv::Vec4d right_line;
  cv::Point right_b;
  double right_m;
  std::vector<cv::Point> left_pts;
  cv::Vec4d left_line;
  cv::Point left_b;
  double left_m;


  if (right_flag == true) {
    for (auto i : left_right_lines[0]) {
      ini = cv::Point(i[0], i[1]);
      fini = cv::Point(i[2], i[3]);

      right_pts.push_back(ini);
      right_pts.push_back(fini);
    }

    if (right_pts.size() > 0) {
      cv::fitLine(right_pts, right_line, CV_DIST_L2, 0,0.01,0.01);
      right_m = right_line[1] / right_line[0];
      right_b = cv::Point(right_line[2], right_line[3]);
    }
  }

  if (left_flag == true) {
    for (auto j : left_right_lines[1]) {
      ini2 = cv::Point(j[0], j[1]);
      fini2 = cv::Point(j[2], j[3]);

      left_pts.push_back(ini2);
      left_pts.push_back(fini2);
    }

    if (left_pts.size() > 0) {
      cv::fitLine(left_pts, left_line, CV_DIST_L2, 0,0.01,0.01);
      left_m = left_line[1] / left_line[0];
      left_b = cv::Point(left_line[2], left_line[3]);
    }
  }

  int ini_y = inputImage.rows;
  int fin_y = 470;

  double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
  double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

  double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
  double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

  cv::line( inputImage, cv::Point(right_ini_x, ini_y), cv::Point(right_fin_x, fin_y), cv::Scalar(0,255,0), 3, CV_AA);
  cv::line( inputImage, cv::Point(left_ini_x, ini_y), cv::Point(left_fin_x, fin_y), cv::Scalar(0,255,0), 3, CV_AA);

  cv::namedWindow("Lane", CV_WINDOW_AUTOSIZE);
  cv::imshow("Lane", inputImage);

  output[0] = cv::Point(right_ini_x, ini_y);
  output[1] = cv::Point(right_fin_x, fin_y);
  output[2] = cv::Point(left_ini_x, ini_y);
  output[3] = cv::Point(left_fin_x, fin_y);

  return output;
}
