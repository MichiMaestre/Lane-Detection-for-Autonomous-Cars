# Lane Detection for Autonomous Cars
[![Build Status](https://travis-ci.org/MichiMaestre/Lane-Detection-for-Autonomous-Cars.svg?branch=master)](https://travis-ci.org/MichiMaestre/Lane-Detection-for-Autonomous-Cars)
[![Coverage Status](https://coveralls.io/repos/github/MichiMaestre/Lane-Detection-for-Autonomous-Cars/badge.svg?branch=master)](https://coveralls.io/github/MichiMaestre/Lane-Detection-for-Autonomous-Cars?branch=master)

# Overview
The objective of this project was to design and develop a lane detection algorithm for autonomous vehicles applications. The self driving car market is growing at a very fast pace. Many companies are working in this problem trying to solve every aspect of it, so that autonomous cars can drive safely on the roads. It is a very complex problem due to the many aspects that it relies on: robotics, path planning, navigation, computer vision, mechanics, etc.

This project is focused in the computer vision aspect of it, a crucial module. If an automated car is going to drive around unpredictable environments, it has to be able to perceive and detect every small detail that surrounds it.

So for this project, a lane detection algorithm is proposed as part of the perception component for a self driving vehicle. By using a video feed input of a car driving on the highway, the algorithm will detect where the lane is so that the car can use its location to avoid getting out of the it. It will also be able to predict any turn on the road to secure a good tracking of the lane. The following subsection of the Overview will explain the pipeline of the algorithm step by step. 

### Algorithm
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

# Dependencies

# How to build

# Running the tests

# Doxygen Documentation

# Solo Iterative Process
Since this is a single programmer project, the Solo Iterative Process (SIP) is used to manage it. A product backlog, iteration backlog and work log(time log and code defect log) are used as structure of the whole project. The following link contains these logs. They will be updated through the whole development of the project.

https://docs.google.com/spreadsheets/d/1wAc0GReN9C3EdXlg6FTdIYsp7ub-Ah-TXP2X_KxtHys/edit?usp=sharing

The project will consist of two iterations:
* The release for Iteration 1 will be a functioning code which detects lanes in the input video.
* Iteration 2 will release the unit tests, Doxygen documentation, the finalized project and a well documented README file with instructions on how to build everything. 

# License
