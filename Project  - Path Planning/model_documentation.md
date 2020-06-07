## Model Documentation

This file will describe the documentation and design decisions for Project - Path Planning as part of the Udacity Self-Driving Car Engineer Nanodegree Program

### Goals

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Structure

In order to correctly create valid paths for execution we do the following steps:

- Validating and parsing incoming message data
- Establishing flags for different actions
- Utilizing sensor fusion to determine if we must switch lanes
- Utilizing sensor fusion to determine if lane switch is viable and safe
- Switching lane
- Adjusting our Velocity
- Creating new Path
- Utilizing Spline to smooth out the path
- Sending path to Simulation

### Validating and parsing incoming message data

This particular step can be observerd starting from lines 90 to lines 124. The template code provided has some validation checks to ensure that the data is of the appropriate size and the appropriate type. We have some helper functions such as 'hasData' that checks to ensure that the incoming data is json data and it is not empty.

Following those validation checks, we parse the data by creating a json object that holds the car's x, y positions, it's sd coordinate frenet coordinates. The car's yaw angle in the map, and the speed it is currently going at. Some additional information of importance is that we are also provided trhe previous path that we sent to the sim, and the sensor fusion data of the vehicles around us.

### Establishing flags for different actions

In order to accurately determine the different actions that our car can/must take. I keep a selection of different flags that allow us to transition between different actions: switching lanes, lane availability, acceleration/deceleration, etc.

The following flags are established in the code.

- is_too_close
- prepare_for_lane_change
- ready_for_lane_change
- is left_lane_available
- is_right_lane_available
- has_switched_lane
- on_lane_switch_cooldown

### Utilizing sensor to fusion to determine if we must switch lanes

During this particular step, our goal is to utilizing sensor fusion to specifically determine if there is a car in front of us. Based on this we can turn the following flags to true: is_too_close, prepare_for_lane change.

The logic for this behaviour can be observed in the 'checkVehicleInFront' function which does the following: Goes through the sensor fusion vehicles and checks if they are inside out lane. We then proceed to ensure that the vehicles in the lane are vehicles that are only in front of u. We do this by checking the serret measurement of the vehicle and comparing it to ours. The final check is if the vehicle in front of us is within the safety margin that we have set in our helpers.cpp file. If both of these conditions are true, then we can make the claim that the vehicle is in front of us, and it is too close for safety.

The final step is to enable the flags is 'is_too_close' and 'prepare_for_lane_change'. Additionally, we also keep save the speed of that vehicle so that we can later use it when calculating our acceleration.

### Utilizing sensor fusion to determine if lane switch is viable and safe

The logic for this step is only triggered if the 'prepare_for_lane_change' is set to true by the previous 'checkVehicleInFront' logic. The logic executes the following behaviours. We iterate through all the sensor fusion objects and initially check for only the vehicles that are to the left and right of us. Since we are in the state of determining a valid lane switch we only checks our left/right lanes.

Following this lane check, we check the vehicles on the other lanes that are x serret units in front of us. This allows us to determine if a lane switch in that particular lane is worth doing, if there is a blocking vehicle in front of us in that lane. That lane is not valid if there is a vehicle that will just be blocking us just like the current lane we are in.

The next check determines if there is enough enough space in that lane for us to make the lane switch. We do this action again by comparing the sensor fusion's vehicles serret units to our car's, taking into account the safety-marging distance that we set in our 'helper.h' file. If this condition is not met, then we determine this lane switch to not be a valid lane to switch to.

If a lane is available we turn on the flag for 'ready_for_lane_change' to be able to proceed with lane switching.

### Switching lane

The core logic for switching lane can be observed in 'updateDrivingLane'. However, it must be noted that I am using a cooldown system for lane switching to solve a few issues that will be described in more detail.

Lane switching first checks to ensure that we are not on 'on_lane_switch_cooldown'. Following that initial check we check which lanes we have previously determined to be safe. If we determined that the left lane is safe, then we will change our current 'lane' to be lane = lane - 1. Meaning that we are now going into another lane. The same behaviour can be obeserved for switching to the right, where we increa lane by 1 instead of decreasing our lane.

Some of the initial observed behaviour with lane switching resulted in two particular problems:

1. When there were two vehicles in front of our car that were approximately going the same speed and on the same serret units. When deciding that a lane switch was valid, as soon as it started the switch to the other lane with a similar vehicle in front of it, it would then decide to go back to it's previous lane. This causes the car to get stuck deciding between both lanes, and ultimately staying in the middle of two lanes because it was trying to switch to both lanes back and forth.

2. Sometimes when switching lanes, from let's say the fast lane on the left. It would switch over to the right, find a vehicle in front of it, and then proceed to switch over to the right. While this was perfectly and smooth behaviour that followed the rules. The actual movement of the car caused it to look reckless, since it was essentially switching two lanes in one smooth motion.

In order to solve this particular problem, I implemented a cooldown system for lane switching. Essentially every single time that the car decided to switch lane, it would go on a timer and wait an x amount of time before proceeding to switch to another lane. This solve the indecisiveness behaviour that it previously showed, and would no longer do a two lane switch in a single motion.

The logic for this can be observed in lines 224-227 and 326-331. Essentially we have a flag 'on_lane_switch_cooldown' which is set to true everytime we do a lane switch. And it gets turned off after x seconds have elapsed. Therefore enabled the action of lane switching to be valid again.

### Adjusting our velocity

Adjusting the car's velocity is an action that occurs constantly. All the behaviour for this can be observed in 'updateVelocity' . The two main triggers for changing a vehicle's velocity is if we have determined that we are too close to a vehicle indicated by the 'is_too_close' flag, or if our current velocity is less than the maximum speed limit, meaning we have room to accelerate.

Our acceleration/deceleration values can be observed in the helpers class under the 'acceleration', 'deceleration' variables. The main point of interest for this particular function is that I am using a weighted average formula in order to create 'smooth' deceleration. This can be observed under the variable 'calculated_deceleration' where I slowly try to match the speed of the vehicle in front of us by slowly decelerating, and falling back to a hardcoded deceleration value when this weighted average deceleration is deemed too dramatic in change that would cause jerk to increase.

### Creating new path

The creation of new path involve the vectors 'pts_x' and 'pts_y'. These are used to hold the coordinates for the next coordinates points. In lines 245 we can observe that we check if we have a 'prev_size'. The 'prev_size' contains the value of the previous waypoints that we sent the simulator. If the size is less than 2, that means that we are just starting up the simulation and we have on previous paths used. If that is not the case and we do have previous paths, then we use the last 2 previous points as our initial points. This is done so that when we calculate things such as switching from one lane to another, our starting point is our current lane.

Following that, we calculate the future points with the function 'getCartesian'. We set our points to be 30-60-90 serret units in front of us. These points are then added for botht he x and y vectors. At this point we have 5 total points. 2 points from the last points sent to the sim, and 3 points for our future direction.

The next step can be found in line 284-291 where we go through all the current points and ensure that we take into account our current angle. This ensure that the points are appropriately shifted based on the current direction we are heading.

### Utilizing spline to smooth out the path

In order to generate all 50 points that we sent to the sim, we rely on the usage of the 'spline.h' class that produces smooth curvature based on the points supplied. This can be observed starting from around line 309. We create the final vector locatoin that will hold the points that will be sent to simulation, the vectors are 'next_x_vals' and 'next_y_vals'.

We start by populating these values with the previous path's values and then using the spline and calculated future points with the function 'setNextWayPoints'. It is important to note that the loop inside this function runs 50 times on the initial time, and then after that initial run it will run a single time. The reasoning for this is because in the beggining we have no previous data, but afterwards we accumulate newer data. Other things to note in the approach to generate path is that we are for the most part utilizing all of the previous points from the path and only appending a single point at the very end. THe reason for this is that we want to ensure smooth transition in our behaviours. We do take into account the fact that we are running at 50 frames per second which is noted in line 390 where we calculate our next measurement in the future based on our current speed.

By slowly appending only the last point, you can observe in the simulator that there will be sections of the road where the green waypoints are on top of the vehicle in front of us because it is still updating the new waypoints with that information. There is definitely rtoom for improvement in this method, possibly prioritizing a more dynamic update method under certain conditions.

### Sensing path to Simulation

This step is very self explanatory and can be observed around line #319. The main points of interest is that the data is packaged a json object that contains the following two values 'next_x' and 'next_y' which we assign the vector values of 'next_y_vals' and 'next_y_vals' which contain the next waypoints for the simulator to execute.
