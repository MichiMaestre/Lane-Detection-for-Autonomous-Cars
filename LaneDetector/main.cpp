#include <iostream>
#include "../include/LaneDetector.hpp"
#include "../LaneDetector/LaneDetector.cpp"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>


int main() {

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

    while(i < 540) {

      if (!cap.read(frame))
        break;

      // Start image processing
      // Denoise the image using a Gaussian filter
      img_denoise = lanedetector.deNoise(frame);

      // Detect edges in the image
      img_edges = lanedetector.edgeDetector(img_denoise);
//      cv::imshow("Edges", img_edges);

      // Mask the image so that we only get the ROI
      img_mask = lanedetector.mask(img_edges);

      // Obtain Hough lines in the cropped image
      lines = lanedetector.houghLines(img_mask);

      //Separate lines into left and right lines
      left_right_lines= lanedetector.lineSeparation(lines, img_edges);

      //Apply regression to obtain only one line for each side of the lane
      lane = lanedetector.regression(left_right_lines, frame);

      // Predict the turn by determining the vanishing point of the the lines
      turn = lanedetector.predictTurn();

      // Plot lane detection
      lanedetector.plotLane(frame, lane, turn);

      i += 1;
      cv::waitKey(20);
    }
    return 0;
}
