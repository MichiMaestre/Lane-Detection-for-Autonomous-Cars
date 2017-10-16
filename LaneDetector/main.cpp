#include <iostream>
#include "../include/LaneDetector.hpp"
#include "../LaneDetector/LaneDetector.cpp"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[]) {

    if (argc != 2) {
      std::cout << "Not enough parameters" << std::endl;
      return -1;
    }

    std::string source = argv[1];
    cv::VideoCapture cap(source);
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
    int flag_plot = -1;
    int i = 0;

//    //Video Object
//    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
//    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
//    cv::Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
//    cv::VideoWriter oVideoWriter ("/home/michi/Desktop/output_video.avi", CV_FOURCC('P','I','M','1'), 25, frameSize,true);

    while(i < 540) {

      if (!cap.read(frame))
        break;

      // Start image processing
      // Denoise the image using a Gaussian filter
      img_denoise = lanedetector.deNoise(frame);

      // Detect edges in the image
      img_edges = lanedetector.edgeDetector(img_denoise);

      // Mask the image so that we only get the ROI
      img_mask = lanedetector.mask(img_edges);

      // Obtain Hough lines in the cropped image
      lines = lanedetector.houghLines(img_mask);

      if(!lines.empty()) {

        //Separate lines into left and right lines
        left_right_lines= lanedetector.lineSeparation(lines, img_edges);

        //Apply regression to obtain only one line for each side of the lane
        lane = lanedetector.regression(left_right_lines, frame);

        // Predict the turn by determining the vanishing point of the the lines
        turn = lanedetector.predictTurn();

        // Plot lane detection
        flag_plot = lanedetector.plotLane(frame, lane, turn);

        i += 1;
        cv::waitKey(25);
      }
      else
        flag_plot = -1;
    }
    return flag_plot;
}
