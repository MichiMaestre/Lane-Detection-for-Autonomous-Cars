/** MIT License
Copyright (c) 2017 Miguel Maestre Trueba
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *@copyright Copyright 2017 Miguel Maestre Trueba
 *@file test.cpp
 *@author Miguel Maestre Trueba
 *@brief Test cases for code coverage.
 */

#include <opencv2/highgui/highgui.hpp>
#include <gtest/gtest.h>
#include<iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "../include/LaneDetector.hpp"

/**
 *@brief Function very similar to demo.cpp. It tests only one iteration of the algorithm for a single image.
 *@param video is a flag that selects the demo video or an image without lanes for testing purposes
 *@param frame_number gives the exact frame number of the input video
 *@return flag_plot tells if the demo has sucessfully finished
 */
int testing_lanes(int video, int frame_number) {
    LaneDetector lanedetector;  // LaneDetector class object
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

    // Select demo video or image without any lines
    if (video == 1) {
      cv::VideoCapture cap("project_video.mp4");
      cap.set(cv::CAP_PROP_POS_FRAMES, frame_number);
      cap.read(frame);
    } else {
        frame = cv::imread("gradient1.png");
    }

    // The input argument is the location of the video
    img_denoise = lanedetector.deNoise(frame);
    // Detect edges in the image
    img_edges = lanedetector.edgeDetector(img_denoise);
    // Mask the image so that we only get the ROI
    img_mask = lanedetector.mask(img_edges);
    // Obtain Hough lines in the cropped image
    lines = lanedetector.houghLines(img_mask);
    if (!lines.empty()) {
      // Separate lines into left and right lines
      left_right_lines = lanedetector.lineSeparation(lines, img_edges);
      // Apply regression to obtain only one line for each side of the lane
      lane = lanedetector.regression(left_right_lines, frame);
      // Predict the turn by determining the vanishing point of the the lines
      turn = lanedetector.predictTurn();
      // Plot lane detection
      flag_plot = lanedetector.plotLane(frame, lane, turn);
    } else {
        flag_plot = -1;
    }

    return flag_plot;
}

/**
 *@brief Test case to test if lane is detected and if the lane is turning left.
 */
TEST(LaneTest, lane_detected) {
  EXPECT_EQ(testing_lanes(1, 3), 0);
}

/**
 *@brief Test cases to test if lane is detected and if the lane is going straight.
 */
TEST(LaneTest, no_turn) {
  EXPECT_EQ(testing_lanes(1, 530), 0);
}

/**
 *@brief Test cases to test if lane is detected and if the lane is turning right.
 */
TEST(LaneTest, right_turn) {
  EXPECT_EQ(testing_lanes(1, 700), 0);
}

/**
 *@brief Test cases to test if lane is not detected at all.
 */
TEST(LaneTest, lane_not_detected) {
  EXPECT_EQ(testing_lanes(0, 1), -1);
}
