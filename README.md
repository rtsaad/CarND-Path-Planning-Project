# Path Planning for the Udacity Simulator

This project consists of a c++ implementation of a Path Planner to control the car using the Udacity self driving simulator. The main goal of this project is to develop a c++ controller that successfully drives the vehicle around the track (Udacity simulator). Figure 1 depicts the car being controlled by the controller. 

![alt text][image1]

[//]: # (Image References)

[image1]: images/example.png "Model Predictive Control"
[image2]: images/mpc_final.png "MPC Graph Plot"
[image3]: images/.png ""

## 1.Access 

The source code for this project is available at [project code](https://github.com/rtsaad/CarND-Path-Planning-Project).

## 2.Files

The following files are part of this project:  
* main.cpp:  main file that integrates the controller with the simulator;
* vehicle.cpp: vehicle class definition;
* cost_function.cpp: cost_function class to compute the optimun path;


### 2.1 Dependency

This project requires the following packages to work:
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

## 3.How to use this project

To run this project, you first need to compile the code. After the code is compiled, please, run the Udacity simulator and the path planner binary created at the build folder.

### 3.1 Compiling and Running

The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning
6. Run the Udacity Simulator (./term3_simulator)

## 4.Path Planning 


### 4.1 Finite State Machine



### 4.2 Cost Functions



