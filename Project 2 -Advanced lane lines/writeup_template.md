## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/calibration/test_calibration_before.jpg "Distorted"
[image7]: ./output_images/calibration/test_calibration_after.jpg "Undistorted"
[image8]: ./output_images/color_spaces/color_spaces.png "Color Spaces"
[image9]: ./output_images/perspective_transform/perspective.png "Perspective Transforms"
[image2]: ./test_images/test1.jpg "Road Transformed"
[image3]: ./examples/binary_combo_example.jpg "Binary Example"
[image4]: ./examples/warped_straight_lines.jpg "Warp Example"
[image5]: ./output_images/lane_identification/lane_identification.png "Fit Visual"
[image6]: ./output_images/curvature_radius/curvature_radius.png "Output"
[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for the calibration can be found in the `calibration_utils.py` class

Calibration starts by first identifying the grid size for the particular chess board size we will be using. For this work, we have chosen to go with a 6 by 9 grid board. This can be observed in lines 42-43.

The calibration of the iamge is done in the following steps.

1. Reading the image file utilizing `cv2.imread()`
2. Converting the image to grayscale using `cv2.cvtColor()`
3. Running the `findChessboardCorners()` algorithm and specifying the correct gridsize
4. Run the `calibrateCamera()` command to obtain the camera matrix and distort coefficients
5. Execute the `undistort()` function algorithm by passing the previously obtained matrix and coefficients. 

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

Below can be observed examples of a before and after distortion correction. 

Pay close attention to the `deer warning sign` and the sides of the `car hood` to notice the effects.

![alt text][image1]

![alt text][image7]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

The usage of color transform can be found in the `binarization_utils.py` class. 

The following color spaces were used inorder to obtain different features that were eventually combined to represent a better final output.

- HSV Color space was ued to obtain the yellow lane accurately. Observation showed that we received better results on channel V.
- Histogram equalization was used to obtain the initial rough highlights of the lanes.
- Sobel algorithm was employed for edge detection, this of course used the grayscale space.
- Morphology Transformation were used to get rid of the gaps observed in the frames. MORPH_CLOSURE was the types of morphed employed.

An example of each of the color spaces above can be observed below:

![alt text][image8]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for perspective transformation can be found in the `perspective_utils.py` file.

In here we can observed in the `get_birdeye_frame()` function found in line 10- 47. In this particular function we can see that I have selected some points that give me some desirable results.

Most of the logic here is handled by the usage of the `getPerspectiveTransform()` and `warpPerspective()` functions.

Below we can observe some of the results obtained from this.

![alt text][image9]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

The code relating to lane pixel positioning can be observed in `line_utils.py`. Particularly the function `get_fits_by_sliding_windows()` handles this.

The approach taking in this method involved obtaining the the peak locations of the histogram found in the binary images by running the `get_binarize_frame()` obtained in previous steps. Obtaining the peaks allows us to decide the pixels that pertain to which lane line (right or left).

Once we've obtained the top two peaks we slide the windows and make those peaks our new starting point and continue from there. This way we build an accurate representations for both lanes.

Some optimization is handled in `get_fits_by_previous_fits()`. In order to avoid having to re-compute the entire pixel length of a new frame, when we're moving towards a new frame, we check if our previous frames had detected both a right and left lane. If this is true then we proceed to run the optimized method. This check can be observed in `main.py` line 112.


![alt text][image5]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

For obtaining the radius of curvature, we rely on using numpy's `polyfit` function that allows us to obtain the necessary values to use in the `radius of curvature` formula introduced in the course. This can be observed in the `line_utils.py` class where we calculate the curvature in `curvature()`.


For the step regarding the vehicle position, we are given some factual information that is key to figure out this problem.

- The camera is mounted somewhere on the car hood. As we are able to see glimpses of it on the frames
- The camera position is fixed and there isn't any visible variation. 

Based on these 2 facts we do the computation of the offset in `computer_offset_from_center()` function found in `main.py`.


#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.


![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

My video can be found in this project directory labeled `final_video.mp4`

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

So far we have implemented various techniques that allow us to more accurately detect lines. The biggest flaws in all of these approaches is the usage of hardcoded parameters. In order to solve this sort of issue, I believe more advanced machine learning techniques must be incorporated. I believe by training a neural net with some of the data we've gathered or from other sources such as simulation we'll be able to more accurately solve lane detection. 