# CarND-MPC-Project-master

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Self-Driving Car Engineer Nanodegree Program

## Concepts

Model Predictive Control (MPC) is an advanced method of process control that is used to control a process while satisfying a set of constraints. 

Advantage:
- The MPC optimizes a finite time-horizon, but only implements the current timeslot and then optimizing again, repeatedly. That it allows the current timeslot to be optimized, while keeping future timeslots in account. 

- Also MPC has the ability to anticipate future events and can take control actions accordingly.

## Steps

- Set the N and dt
- Fit the ploynomial to waypoints
- Calculate initial cte and orientation error value
- Define the cost function
- Define the model constraints


## Question

### The Model

- the state and actuators

The state consists of sytem variables and errors references: `[x,y,psi,v,cte,epsi]`. The x and y stand for the vehicle position, psi the vehicle orientation, v the vehicle speed, cte and epsi stand for the cross track error and orientation error of the vehicle related to the reference.

- The update equations

![image](./Images/Equations.png)

`Lf` measures the distance between the front of the vehicle and its center of gravity. `f(x)` is the evaluation of the polynomial f at point `x` and `psidest` is the tangencial angle of the polynomial `f` evaluated at `x`.

### Timestep Length and Elapsed Duration (N & dt)

First, we should detemine a reasonable range for T`(T = N * dt)`, which makes cte as small as possible. then we tune the `N` and `dt`.

- T

T should not be too large or too small, too large, the predict horizon is too long,it is difficult to predict the actual trajectory of the predicted trajectory, too small, response time of the model is too short. In the actual simulation process, the value of 1.0 to 1.5 works well, and finally I chooses 1.0.(The speed setting is faster, and the time span is set larger, and the probability of failure is higher.)

- N and dt

I think dt should be consistent with the delay set by the system, We predict the state of the system based on the delay, and we should also calculate the optimal path based on the delay. The T is 1.0, and the dt is 0.1s, so the N is 10.

### Polynomial Fitting and MPC Preprocessing

We need to convert the measurement point from the original coordinate system to the car coordinate system. then we use the helper function `polyfit` to perform polynomial matching.
```
for(unsigned int i=0; i < ptsx.size(); i++){
	double shift_x = ptsx[i] - px;
	double shift_y = ptsy[i] - py;
	ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
	ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
}
```
### Model Predictive Control with Latency

The model sets a 100 delay. so we predict the next state before calling the MPC solver. It can be acoomplished using the Model equations.
```
double x1=0, y1=0,  psi1=0, v1=v, cte1=cte, epsi1=epsi;
//predict
x1 += v * cos(0) * dt;
y1 += v * sin(0) * dt;       
cte1 +=   v * sin(epsi1) * dt;
epsi1 += - v * steer_value / Lf * dt;//steer_value is negative 
psi1 += - v/Lf * steer_value * dt;//steer_value is negative
v1 += throttle_value * dt;  
```

### The Cost function and Constraints

We sample the state of the car at time t and minimize the cost in the time domain `[t, t+T]` without exceeding the parameter constraints to find the optimal trajectory.

- Cost function

We build the cost function mainly based on cte and epsi itself or parameters that may cause changes in cte and epsi.

```
fg[0] = 0;
//Cost related to the reference state.
for (unsigned int t = 0; t < N; t++) {
	fg[0] += 2000 * CppAD::pow(vars[cte_start + t], 2); 
      	fg[0] += 2000 * CppAD::pow(vars[epsi_start + t], 2);
      	fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
//Minimize the use of actuators.
for (unsigned int t = 0; t < N - 1; t++) {
      	fg[0] += 50 * CppAD::pow(vars[delta_start + t], 2);
      	fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
	}
//Minimize the value gap between sequential actuations.
for (unsigned int t = 0; t < N - 2; t++) {
      	fg[0] += 5 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      	fg[0] += 5 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	}
```
> Reasons for lower factor of speed and acceleration : The setting of the reference speed is to make the vehicle always move toward the destination. In the actual simulation process, I found that if the coefficient is set too large, the vehicle will keep driving at a high speed all the time, and the speed is very fast during the extreme turning, which is not in common sense. : When driving in a straight line, the speed is faster, when slow speed is required, the vehicle should be decelerated, so I chose a smaller coefficient to allow the vehicle to slow down.

- Constraints

Acceleration is -1 to 1 and yaw angle is -25 degree to 25 degree.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`. 

## Result

Under the current parameters of my settings(the reference speed is 80), the car can run several times successfully, the reference speed becomes higher, and it may fail. The lower the reference speed setting, the longer the normal time.

![image](./Images/run.png)

## Unresolved issue :

- The delay of the model

The delay of the vehicle depends on many aspects. The calculation performance of the model, the time from the receipt of the command to the actual output of the vehicle will affect the delay. Therefore, the delay will be different for each car and the different stages of the same car. How to determine the appropriate delay?

- The possibility of rushing out of the runway

With the same parameters, the car is not completed successfully every time. And as the car travels longer, it will eventually rush out of the runway, the longest time is about twenty minutes, what is the reason for the failure?

- The car swayed sharply at the beginning

In the beginning, the car is at the center, the largest percentage of the loss function should be speed, cte and psi are small, the car's reasonable response is accelerated, why is it swinging around?

- The calculated best trajectory will change dramatically in some cases

Since the car cannot change drastically in some respects, the acceleration and yaw angular accelerations are all set to a range of variation, and changes in these quantities are added to the loss function. But in some places, especially in corners, the trajectory will suddenly deviate from the right position. Although most of the cases are corrected at the next moment, sometimes the car will be washed out of the runway. The faster the speed, the more likely it is. Auto-driving requires 100% safety, which is absolutely not allowed. What caused these changes?
