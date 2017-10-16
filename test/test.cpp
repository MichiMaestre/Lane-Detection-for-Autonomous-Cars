#include <gtest/gtest.h>
#include<iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "../include/LaneDetector.hpp"
//#include "../include/LaneDetector_test.hpp"

int testing_lanes(int video, int frame_number) {
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
    int flag_plot = -1;

    if (video == 1) {
      cv::VideoCapture cap("project_video.mp4");
      cap.set(cv::CAP_PROP_POS_FRAMES, frame_number);
      cap.read(frame);
    }
    else
      frame = cv::imread("gradient1.png");

    img_denoise = lanedetector.deNoise(frame);
    img_edges = lanedetector.edgeDetector(img_denoise);
    img_mask = lanedetector.mask(img_edges);
    lines = lanedetector.houghLines(img_mask);
    if(!lines.empty()) {
      left_right_lines= lanedetector.lineSeparation(lines, img_edges);
      lane = lanedetector.regression(left_right_lines, frame);
      turn = lanedetector.predictTurn();
      flag_plot = lanedetector.plotLane(frame, lane, turn);
    }
    else
      flag_plot = -1;

    return flag_plot;
}


TEST(LaneTest, lane_detected) {
  EXPECT_EQ(testing_lanes(1, 3), 0);
}

TEST(LaneTest, no_turn) {
  EXPECT_EQ(testing_lanes(1, 530), 0);
}

TEST(LaneTest, right_turn) {
  EXPECT_EQ(testing_lanes(1, 700), 0);
}

TEST(LaneTest, lane_not_detected) {
  EXPECT_EQ(testing_lanes(0, 1), -1);
}
