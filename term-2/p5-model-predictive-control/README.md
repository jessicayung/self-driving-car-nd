# Model Predictive Control Project
CarND-Controls-MPC, Self-Driving Car Engineer Nanodegree Program

The objective of this project is to use model predictive control to enable a car to drive around a track in a simulator.

---
## Table of contents

This README contains information on:

1. MPC Model
    - Tuning parameters N, dt
2. Files in this repo
3. Dependencies
4. Basic Build Instructions
5. Notes from Udacity's README


## 1. MPC Model

We will first describe the state, actuator and update equations (ingredients in the model) and then go on to describe the MPC model setup and loop. We will then discuss tuning parameters.

#### State [x,y,ψ,v]
Our state can be described using four components:
1. x: the x-position
- y: the y-position
- ψ: the vehicle orientation
- v: the velocity

#### Actuators [δ, a]
* An actuator is a component that controls a system. Here we have two actuators: 
    * δ (the steering angle) and 
    * a (acceleration, i.e. throttle and brake pedals).

#### Update equations (Vehicle Dynamics)
* x = x + v\*cos(ψ)\* dt 
* y = y + v sin(psi) dt
* v=v+a∗dt
    * a in [-1,1]
* ψ=ψ+(v/L_f)*δ∗dt

#### MPC Setup:
1. Define the length of the trajectory, N, and duration of each timestep, dt.
    * See below for definitions of N, dt, and discussion on parameter tuning.
* Define vehicle dynamics and actuator limitations along with other constraints.
    * See the state, actuators and update equations above.
* Define the cost function.
    * Cost in this MPC increases with: (see `MPC.cpp` lines 80-101)
        * Difference from reference state (cross-track error, orientation and velocity)
        * Use of actuators (steering angle and acceleration)
        * Value gap between sequential actuators (change in steering angle and change in acceleration).
    * We take the deviations and square them to penalise over and under-shooting equally.
        * This may not be optimal.
    * Each factor mentioned above contributed to the cost in different proportions. We did this by multiplying the squared deviations by weights unique to each factor.

#### MPC Loop:
1. We **pass the current state** as the initial state to the model predictive controller.
* We call the optimization solver. Given the initial state, the solver will ***return the vector of control inputs that minimizes the cost function**. The solver we'll use is called Ipopt.
* We **apply the first control input to the vehicle**.
* Back to 1.

*Reference: Setup and Loop description taken from Udacity's Model Predictive Control lesson.*


#### N and dt
* N is the number of timesteps the model predicts ahead. As N increases, the model predicts further ahead.
* dt is the length of each timestep. As dt decreases, the model re-evaluates its actuators more frequently. This may give more accurate predictions, but will use more computational power. If we keep N constant, the time horizon over which we predict ahead also decreases as dt decreases.

#### Tuning N and dt
* I started with (N, dt) = (10, 0.1) (arbitrary starting point). The green panth would often curve to the right or left near the end, so I tried increasing N so the model would try to fit more of the upcoming path and would be penalised more if it curved off erratically after 10 steps.
* Increasing N to 15 improved the fit and made the vehicle drive smoother. Would increasing N further improve performance?
* Increasing N to 20 (dt = 0.1) made the vehicle weave more (drive less steadily) especially after the first turn. 
    * The weaving was exacerbated with N = 50 - the vehicle couldn't even stay on the track for five seconds. 
* Increasing dt to 0.2 (N = 10) made the vehicle too slow to respond to changes in lane curvature. E.g. when it reached the first turn, it only started steering left when it was nearly off the track. This delayed response is expected because it re-evaluates the model less frequently. 
* Decreasing dt to 0.05 made the vehicle drive in a really jerky way.
* So I chose N = 15, dt = 0.1.
* It would be better to test variations in N and dt more rigorously and test different combinations of N, dt and the contributions of e.g. cross-track error to cost. 
* It would also be good to discuss variations in N and dt without holding N or dt fixed at 10 and 0.1 respectively.

### Latency
* If we don't add latency, the car will be steering and accelerating/braking based on what the model thinks it should've done 100ms ago. The car will respond too slowly to changes in its position and thus changes in cost. 
	* It may not start steering e.g. left when it goes round a curve, leading it to veer off the track. 
	* Likewise, it may continue steering even when the path stops curving. 
	* The faster the vehicle speed, the worse the effects of latency that is unaccounted for.
* **Implementation**: We used the kinematic model to predict the state 100ms ahead of time and then feed that predicted state into the MPC solver.
	* We chose 100ms because that's the duration of the latency. That is, we try to predict where the car will be when our instructions reach the car so the steering angle and throttle will be appropriate for when our instructions reach the car (100ms later).
	* The code can be found in `main.cpp`.

### Other comments
* Strangely, when tackling the case where there is 100ms latency, predicting the state 100ms ahead gave a worse outcome than not predicting the state.

## 2. Files in this repo

Key files:
* `src/main.cpp`
    * Interacts with simulator and calls MPC.
* `src/MPC.cpp`
    * Implements MPC.

## 3. Dependencies

 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## 4. Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## 5. Notes from Udacity's README
### Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

### Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

### Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

### Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

### Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
