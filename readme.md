# Lane Detection for Autonomous Cars
[![Build Status](https://travis-ci.org/MichiMaestre/Lane-Detection-for-Autonomous-Cars.svg?branch=master)](https://travis-ci.org/MichiMaestre/Lane-Detection-for-Autonomous-Cars)
[![Coverage Status](https://coveralls.io/repos/github/MichiMaestre/Lane-Detection-for-Autonomous-Cars/badge.svg?branch=master)](https://coveralls.io/github/MichiMaestre/Lane-Detection-for-Autonomous-Cars?branch=master)

# Overview
A lane detection algorithm is proposed as part of the perception component for a self driving vehicle. By using a video feed input of a car driving on the highway, the algorithm will detect where the lanes are so that the car can use its location to avoid getting out of the lane. It will also be able to predict any turn on the road to secure a good tracking of the lane.
The following figure shows a frame of the input video:

![inputImage](https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars/blob/master/images/inputImage.png)

# Solo Iterative Process
Since this is a single programmer project, the Solo Iterative Process (SIP) is used to manage it. A product backlog, iteration backlog and work log(time log and code defect log) are used as structure of the whole project. The following link contains these logs. They will be updated through the whole development of the project.

https://docs.google.com/a/terpmail.umd.edu/spreadsheets/d/1wAc0GReN9C3EdXlg6FTdIYsp7ub-Ah-TXP2X_KxtHys/edit?usp=sharing

The project will consist of two iterations:
* The release for Iteration 1 will be a functioning code which detects lanes in the input video.
* Iteration 2 will release the unit tests, Doxygen documentation and the finalized project. 

# Algorithm
The proposed algorithm follows a straight forward pipeline with several steps as shown in the following activity diagram. It will iterate through every frame of the video and do the following steps in each of them:
![diagram](https://github.com/MichiMaestre/Lane-Detection-for-Autonomous-Cars/blob/master/UML/initial/Activity_UML.png)

* The first step will be to denoise the image by applying a filter like a Gaussian mask that smooths the image and removes any undesired pixel values that could prevent the correct detection of the lanes. 
* Secondly, an edge detection algorithm is used to detect the lines that form the boundaries of the lanes. Canny or Sobel are examples of edge detection algorithms that can be used.
* Since there is a big chance of unwanted edge detection in other objects of the image, the image will be masked so that it only detects edges in a region of interest. This helps because the road is always going to be in the same location on the image.
* Once the desired edges are detected, the next step will be using the Hough Lines detection algorithm, which will give the location of all the lines in the image.
* By applying basic linear algebra, all the detected Hough lines will be classified as left side lines or right side lines.
* Regression will be applied to the stored left and right lines clusters to obtain only one line for each side. These lines will be the corresponding lane boundaries.
* Lastly, the turn prediction is determined by the location of the vanishing point in the image, formed by the boudaries of the lane.
