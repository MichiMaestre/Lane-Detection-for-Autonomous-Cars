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

//    cv::namedWindow("Edges",CV_WINDOW_AUTOSIZE);
    while(1) {

      if (!cap.read(frame))
        break;

//      cv::imshow("Input", frame);

      // Start image processing
      // Denoise the image using a Gaussian filter
      img_denoise = lanedetector.deNoise(frame);

      // Detect edges in the image
      img_edges = lanedetector.edgeDetector(img_denoise);
//      cv::imshow("Edges", img_edges);

      // Mask the image so that we only get the ROI
      img_mask = lanedetector.mask(img_edges);

      // Obtain Hough lines in the cropped image
      lines = lanedetector.houghLines(frame, img_mask);

      cv::waitKey(35);
    }
    return 0;
}
