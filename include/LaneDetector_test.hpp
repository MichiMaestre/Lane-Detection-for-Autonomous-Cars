#include<iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "../include/LaneDetector.hpp"
#include "../LaneDetector/LaneDetector.cpp"

int testing_lanes() {

//    if (argc != 2) {
//      std::cout << "Not enough parameters" << std::endl;
//      return -1;
//    }

//    std::string source = argv[1];
    cv::VideoCapture cap("/home/michi/Desktop/project_video.mp4");
    if (!cap.isOpened())
      return -1;

    LaneDetector lanedetector;
    cv::Mat frame;
    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;
    std::string turn;
    int i = 0;

    while(i == 1) {

      if (!cap.read(frame))
        break;


      img_denoise = lanedetector.deNoise(frame);
      img_edges = lanedetector.edgeDetector(img_denoise);
      img_mask = lanedetector.mask(img_edges);
      lines = lanedetector.houghLines(img_mask);
      left_right_lines= lanedetector.lineSeparation(lines, img_edges);
      lane = lanedetector.regression(left_right_lines, frame);
      turn = lanedetector.predictTurn();
      lanedetector.plotLane(frame, lane, turn);

      i += 1;
      cv::waitKey(25);
    }
    return 0;
}
