# Lane Detection for Autonomous Cars
[![Build Status](https://travis-ci.org/MichiMaestre/Lane-Detection-for-Autonomous-Cars.svg?branch=master)](https://travis-ci.org/MichiMaestre/Lane-Detection-for-Autonomous-Cars)
[![Coverage Status](https://coveralls.io/repos/github/MichiMaestre/Lane-Detection-for-Autonomous-Cars/badge.svg?branch=master)](https://coveralls.io/github/MichiMaestre/Lane-Detection-for-Autonomous-Cars?branch=master)

## Overview
The objective of this project was to design and develop a lane detection algorithm for autonomous vehicles applications. The self driving car market is growing at a very fast pace. Many companies are working in this problem trying to solve every aspect of it, so that autonomous cars can drive safely on the roads. It is a very complex problem due to the many aspects that it relies on: robotics, path planning, navigation, computer vision, mechanics, etc.

This project is focused in the computer vision aspect of it, a crucial module. If an automated car is going to drive around unpredictable environments, it has to be able to perceive and detect every small detail that surrounds it.

So for this project, a lane detection algorithm is proposed as part of the perception component for a self driving vehicle. By using a video feed input of a car driving on the highway, the algorithm will detect where the lane is so that the car can use its location to avoid getting out of the it. It will also be able to predict any turn on the road to secure a good tracking of the lane. 
The project was developed using C++, CMake and the OpenCV library. The input video was taken from Self-Driving Car Engineer Nanodegree. 

The following subsection of the Overview will explain the pipeline of the algorithm step by step. 

#### Algorithm
The proposed algorithm follows a straight forward pipeline with several steps as shown in the following activity diagram. It will iterate through every frame of the video and do the following steps in each of them:

![diagram](https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars/blob/master/UML/initial/Activity_UML.png)

* The frames of a video are captured. This image inputs are from a video of a car driving on the highway.

![inputImage](https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars/blob/master/images/inputImage.png)

* The first step will be to denoise the image by applying a filter like a Gaussian mask that smooths the image and removes any undesired pixel values that could prevent the correct detection of the lanes. The picture below shows how the the filter blurres the same frame as the one above.

![denoise](https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars/blob/master/images/denoise.png) 

* Secondly, an edge detection algorithm is used to detect the lines that form the boundaries of the lanes. For this, the image has to be converted to grayscale, and then to binary. Once this is done, the edge detection is achieved by applying a row kernel `[-1 0 1]` to each pixel of the image. This kernel is based on the one used in the Lane Departure Warning System developed by Mathworks.

![edges](https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars/blob/master/images/edges.png)

* As it can be seen in the image above, there are lots of unwanted edges being detected. To fix this, the image will be masked so that it only detects edges in a region of interest. This helps because the road is always going to be in the same location on the image. The following figure shows the result of masking the edges image:

![mask](https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars/blob/master/images/mask.png)

* Once the desired edges are detected and the unwanted ones have been removed, the next step is to use the Hough Lines detection algorithm, which gives the location of all the lines in the image. The parameters of the function (rho and theta) were determined by trial and error so that only the most notable lines are detected. As seen in the image, the most obvious lines are the ones being detected.

![hough](https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars/blob/master/images/hough.png)

* By applying basic linear algebra, all the detected Hough lines will be classified as left side lines or right side lines. The lines are classified depending on the value of their slope and where their initial and final points are approximately located with respect to the center of the image.
* Regression will be then applied to the stored left and right lines clusters to obtain only one line for each side. To accomplish this, the least squares method is used. The initial and final points of the lines are used as the data for the regression. The resultant lines will be the corresponding lane boundaries.
* Lastly, the turn prediction is determined by obtaining the location of the vanishing point (formed by the lane boundary lines) with respect to the center of the image.
* Once everything is detected, the results are represented in the input frame, captured at the begnning of the algorithm.

![result](https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars/blob/master/images/Final.png) 

The code can be found in the LaneDetector folder. and the saved output .avi video is in the output_video folder.

## Dependencies
The only dependency for this project is OpenCV 3.2.0. To install it, follow the next steps:

```
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
cd ~/<my_working_directory>
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd ~/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
sudo /bin/bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```

## How to build, run the demo and run the tests
To build and run the demo, follow the next steps (`project_video.mp4` is the video for the demo. `gradient1.png` is just an image without any lanes used during the unit tests for code coverage):

* To build:
```
git clone https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars.git
cd <path to repository>
mkdir build
cd build
cmake ..
Download project video: wget -O project_video.mp4 "https://drive.google.com/uc?export=download&id=0B8dH7dFBBB-nTTREMFdHNnZUVGc"
Download negative image: wget -O gradient1.png "https://drive.google.com/uc?export=download&id=0B8dH7dFBBB-nTXdnS0libDBNeW8"
make
```
* Run the demo:
```
cd build
./LaneDetector/lanes /full/path/to/repository/build/project_video.mp4
```
* Run the tests:
```
cd build
./test/lanes-test
```

## Doxygen Documentation
To generate Doxygen Documentation in HTML and LaTEX, follow the next steps:

```
cd <path to repository>
mkdir <documentation_folder_name>
cd <documentation_folder_name>
doxygen -g <config_file_name>
```
Inside the configuration file, update:
```
PROJECT_NAME = 'your project name'
INPUT = ../LaneDetector ../include ../test
```
Run and generate the documents by running the next command:
```
doxygen <config_file_name>
```
To view the documents easily, access `classLaneDetector.html`, `test_8cpp.html` and `demo_8cpp.html` with your browser.

## Solo Iterative Process
Since this is a single programmer project, the Solo Iterative Process (SIP) is used to manage it. A product backlog, iteration backlog and work log(time log and code defect log) are used as structure of the whole project. The following link contains these logs. They will be updated through the whole development of the project.

https://docs.google.com/spreadsheets/d/1wAc0GReN9C3EdXlg6FTdIYsp7ub-Ah-TXP2X_KxtHys/edit?usp=sharing

The project will consist of two iterations:
* The release for Iteration 1 will be a functioning code which detects lanes in the input video.
* Iteration 2 will release the unit tests, Doxygen documentation, the finalized project and a well documented README file with instructions on how to build everything. 

## License

MIT License

Copyright (c) 2017 Miguel Maestre Trueba

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

