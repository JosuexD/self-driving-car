# CarND-Controls-PID

Self-Driving Car Engineer Nanodegree Program

---

## PID Control

https://gyazo.com/cb52460314f5d53b0fae1285094998e3
Based on the lecture in the course and external material. I came to the understanding of the different effects that each component in the PID controller has based on their different values.

Proportional Gain - This particular component allows us to turn harder the further we are from our desired destination, this is depends on the Cross Track Error (CTE). In terms of actual results. If this number was too large, we would have very excessive steering, on the opposite spectrum, if the value is set too low, it would take a very long time to get back to the center or CTE nearing 0. Below you can observe a gif of an exagerated P value:
https://gyazo.com/78cc8b4cfe99b4c4036b13504fa4e7a7

Derivative Gain - This component allows us to determine how 'quickly' we turn with increased CTE. If this value is too low we enter what we call an 'underdamped' state and we will see a lot of oscillation in our driving. On the opposite spectrum is the derivative gain is too high we get a 'overdamped' behaviour, which would increase the amount of time it takes to get back to a near 0 CTE.

Integral Gain - This component allows us decrease the steady state error in the situation where a natural disaster or a mechanical malfunction makes our car have an offset shift. In order to correct this behaviour, the integral grain sums up all the CTE's and gets multiplied by this integral gain. Some of the observed behaviours with these values are: when the value is too high, the vehicle fluctuates too much based on the accumulated CTE values. If the value is too low on the other hand, then it takes a very long time to fix these dynamic issues. Something to note is that we take advantage of the fact that this component is used for dynamic situations which we don't really see in our particular testing scenario. Therefore allowing us to minimize the importance of this value and therefore assign it a very small value.

## Parameter Choosing

The parameters I chose were entirely depending based on testing trials, and understanding of the initial condition. For example, the simulator already starts us off inside the track, due to this, I concluded the our Proportional Gain should be a small number since we are not starting out with a high CTE and therefore do not need a high adjustment to get bacj on the track.

Another factor that affected the parameter tuning was the fact that there were no real obstacles in the track that might affect the vehicle in a significant manner. Due to no external forces taking part in this simulation, I concluded that our Integral gain was near non-existent. And therefore chose a very very small number as to not affect our steering calculations.

Lastly, the derivative gain calculations again were produced from trial runs. Special attention was paid during curves, which created the increased CTE and where I saw the most effect take place when choosing a derivative value.

## Dependencies

- cmake >= 3.5
- All OSes: [click here for installation instructions](https://cmake.org/install/)
- make >= 4.1(mac, linux), 3.81(Windows)
  - Linux: make is installed by default on most Linux distros
  - Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  - Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
- gcc/g++ >= 5.4
  - Linux: gcc / g++ is installed by default on most Linux distros
  - Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  - Windows: recommend using [MinGW](http://www.mingw.org/)
- [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  - Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  - If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
- Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

- indent using spaces
- set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

- You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

- /ide_profiles/vscode/.vscode
- /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README

A well written README file can enhance your project and portfolio. Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
