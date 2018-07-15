# CarND-P10-Model-Predictive-Controller

## Project Overview

This project consists of implementing a Model Predictive Controller (MPC) in C++.  A simulator provided by Udacity ([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)) generates telemetry and track waypoints data via [uWebSockets](https://github.com/uNetworking/uWebSockets). The MPC controller responds with steering and throttle commands back to the simulator to drive the car around the track. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency. To get this project started, Udacity provides a seed project that could be found [here](https://github.com/udacity/CarND-MPC-Project).

## Implementation

### 1. The Model

**State**
The MPC model defines the vehicle's x and y coordinates, psi - orientation angle, v - velocity, as well as cte - cross track error and epsi - orientation error. The variables x, y, psi and v are generated from the simulator. The cte and epsi values are calculated values, as described in the Polynomial Fitting and MPC Preprocessing section.


**Actuators**
The MPC model defines two actuators: delta - steering angle and a - throttle (which combines acceleration with positive value and brake with negative value). They are obtained from the result of solver and sent back to the simulator (main.cpp line 142-152). Note that when delta is positive we rotate counter-clockwise or turn left. However in the simulator, a positive value implies a right turn or a negative value implies a left turn. So the steering value is mutiplied by -1. Also it is divided by deg2rad(25) before sending back to the server so that the value will be in between -1 and 1.

**Update equations**
The state update are implemented based on the following equations:
![](https://github.com/JuAnne/CarND-P10-Model-Predictive-Controller/blob/master/reference/model_equations.png)

**Cost function tuning**
There are seven multipliers defined (MPC.cpp line 30-36) and tuned based on the performance in the simulator.
I started with small cte and epsi multipliers and found that the car drove off the track. Increasing cost_cte and cost_epsi improved the performance and finally it stayed on the track. Also with small multipliers for the gap between sequential actuations, the car stays on the track but oscillates too much around the center of lane. Increasing the cost_gap_delta and cost_gap_a, which penalizes this behavior (similar to increasing D in a PD controller), the car drove smoothly around the track.


### 2. Timestep Length and Elapsed Duration (N & dt)
*N* is the number of timesteps the model predicts ahead. If N is large, the solver needs to perform more computations before it provides a solution which could lead to inaccurate results in real time.

*dt* is the gap between consecutive time steps. If dt is too small, the car oscillates around the center of the track. If dt is too large, the car drives too far before the actuators react which leads to it going off the road on the curves.

The final values chosen for N and dt are 10 and 0.1. I tried dt values at 0.08, 0.12, 0.15. I found that below 0.1 the car oscillated too much and above 0.15 the car drove off the edge of the track on corners. I also tried N at 12, 15 and 20, which produced erratic behavior.

### 3. Polynomial Fitting and MPC Preprocessing
The x, y received from the simulator are in the global coordinates. They are converted to the vehicle coordinates (main.cpp line 105-114), as the following geometry shows. After the conversion, a third order polynomial is fitted to the x, y coordinates and used to calculate cross track error, cte, and orientation error, epsi (main.cpp line 116-127).

![](https://github.com/JuAnne/CarND-P10-Model-Predictive-Controller/blob/master/reference/coordinates_conversion.png)

### 4. Model Predictive Control with Latency
The simulator latency of 100ms was incorporated to the state equations before calling the model Solver (main.cpp line 130-139).

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
