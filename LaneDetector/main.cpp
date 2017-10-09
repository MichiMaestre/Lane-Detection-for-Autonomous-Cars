#include <iostream>
#include <LaneDetector.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>


int main() {

    cv::VideoCapture cap("/home/michi/Desktop/project_video.mp4");
    if (!cap.isOpened())
      return -1;

    cv::Mat frame;
    cv::namedWindow("Output",CV_WINDOW_AUTOSIZE);
    while(1) {

      if (!cap.read(frame))
        break;

      cv::imshow("Output", frame);
      cv::waitKey(20);

      // Start image processing
    }
    return 0;
}
