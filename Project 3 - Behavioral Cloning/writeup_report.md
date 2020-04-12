**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./images/img1.jpg "sample image 1"
[image2]: ./images/img2.jpg "sample image 2"

[video1]: https://www.youtube.com/watch?v=RsRgAMy8x4g "Final Video"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

[video1]
---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md or writeup_report.pdf summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

The model architecutre used for this particular project was employed by the Nvidia proposed in their 'End to End Learning for Self-Driving Cars's paper.

The approach for this particular paper employs the followings layers:

- Normalization layer
- x5 Convolutional layers
- Flatten layer

The Nvidia model uses 3x3 and 5x5 filter sizes. 

The particular model that I employed for the normalization layer can be observed in line 85. Where a Keras Lambda function is used to normalize our input.

Additionally on line 88 you can observe the Cropping2D method that I used in order to get rid of pixels that covered the environment and the hood of the car. This is to ensure generalizatoin of the model and prevent overfitting.

#### 2. Attempts to reduce overfitting in the model

My particular approach for overfitting was trying out the following techniques:

- Data shuffling
- Data augmentation
- Early Stoppage
- Dropout 
- Simulation Trials

Data shuffling which can be observed in line 60 ensured that the model did not overfit with any particular sequence of images that it may have trained through. By randomizing the order in which the images were trained, I hoped to reduce overfitting. 

Another approach taken was the usage of data augmentation. Part of my early attemps at training my model was that it would not do well with the data I had gathered. I decided to augment the data by flipping the image and the steering angle I had collected as observed in line 51-53. This essentially doubled my training data.

Eearly stoppage was another technique that I learned by researching overfitting. The particular usage of this can be observed in lines 106 and 113. My initial trials leads me to try large amounts of EPOCHS, around 100. This led me to observe how the model error loss would go over large amounts of EPOCHS. Seeing that the model learned very early on allowed me to adjust my parameters to match those vaues. The Early stoppage callback helped ensure that I didn't waste more time than needed training

Dropout layers were added to my model. This was the last step I took to ensure that overfitting would not occur. My particular model hasa dropout of 0.2 and can be observed on line 97. I am adding this after the core Convolutional layers that the NVidia paper had.

Running the simulation was the best way to test how well my model did. Based on the results I was given and the error loss charts that I kept after every run, I adjusted my EPOCHs and varied from using augmented data to not using augmented data until I received a satisfactory result and the car was able to drive through the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 104).

#### 4. Appropriate training data

The training data that I used ended up being the sample data that was provided by Udacity. 

Through this project I did employ collecting my own data, however I did not have much lock in getting a working model purely with my data. This is why I relied on the sample data. 

I did however augment my data by flipping the images in order to double the amount that I had at hand. Some other techniques that were using for the training data was cropping of unecessary pixels. As expressed previously, this can be observed on line 88 where Keras Cropping2D was used to manipulate the image and keep only relevant data.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to leverage the power of Keras and Transfer learning. One of the advantages that I ended up using was the vast amount of research that has already been done, which led me to try out the NVidia's End to End deep learning approach. Once I had tested out this particular architecture, the results I had obtained were great. 

Part of my approach was definitely very iteratively, as it's the nature of machine learning. 

A lot of my decisions relied on observation of error loss and observing when my model decided to start increasing in Error Loss and adjusting my EPOCH parameters. 

After every single training, I ensured to validate my model by actually attempting to run it through the track. This was, afterall the best way to ensure that my model was correctly being trained. 

After hour of parameters tweaking I was finally able to ensure that my model was sufficiently trained well when it could run laps nonestop by staying in the center of the lane.

#### 2. Final Model Architecture

The model as previously mentioned follows NVidia's model. It can be observed in the following lines of the code: 85-101

#### 3. Creation of the Training Set & Training Process

The final training data that was used for this particular project leveraged the data provided by Udacity's sample data. Some of the images can be observed below:

![alt text][image1]
![alt text][image2]

In order to make the best use pre-existing data. I augmented some of the data through the means of image flipping. This is a very simple technique that allows us to correct the left turning bias that the model seemed to favor in the beggining. 

Once all the data was augmented through these techniques, I ensured that I normalized it when feeding it into the models. This can be observed in line 85 where we use a Lambda function to divide the image by 255 (the total amount of values in RGB) and then subtracting by -0.5 in order to set the center of the image to 0. 

The final technique used im the images was cropping pixels. It was observed very early on that certain sections of the road gave me more issues than others, during my initial data collection I had accidently gone off the road in that section. In order to generalize the data and avoid overfitting under certain images, I implementd image cropping to get rid of the environments and the hood of the car. This can be observed in line 88. The thinking behind this is that, if I remove distractors from the model, I can allow it to focus on more general and important aspects of the image that should allow it to better decide how to drive, such as the lane lines and not the hood of the car of the trees.

The data was randomly shuffled using the Keras library and 20% of the data was put into a validation set. This parameter can be tweaked in the params.py file. 