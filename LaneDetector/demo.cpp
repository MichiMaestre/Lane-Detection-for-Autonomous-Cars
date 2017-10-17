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
 *@file demo.cpp
 *@author Miguel Maestre Trueba
 *@brief Demo code that shows the full functionality of the LaneDetector object.
 */

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "../include/LaneDetector.hpp"
#include "../LaneDetector/LaneDetector.cpp"

/**
 *@brief Function main that runs the main algorithm of the lane detection.
 *@brief It will read a video of a car in the highway and it will output the
 *@brief same video but with the plotted detected lane
 *@param The argument is a string to the full path of the demo video
 *@return int flag_plot tells if the demo has sucessfully finished
 */
int main(int argc, char *argv[]) {
    if (argc != 2) {
      std::cout << "Not enough parameters" << std::endl;
      return -1;
    }

    // The input argument is the location of the video
    std::string source = argv[1];
    cv::VideoCapture cap(source);
    if (!cap.isOpened())
      return -1;

    LaneDetector lanedetector;  // Create the class object
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
    int i = 0;

    // Main algorithm starts. Iterate through every frame of the video
    while (i < 540) {
      // Capture frame
      if (!cap.read(frame))
        break;

      // Denoise the image using a Gaussian filter
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

        i += 1;
        cv::waitKey(25);
      } else {
          flag_plot = -1;
      }
    }
    return flag_plot;
}
