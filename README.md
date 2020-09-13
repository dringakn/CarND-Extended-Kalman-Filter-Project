[MainCode]: https://github.com/dringakn/CarND-Extended-Kalman-Filter-Project/blob/master/src/main.cpp
[FusionEKF]: https://github.com/dringakn/CarND-Extended-Kalman-Filter-Project/blob/master/src/FusionEKF.cpp
[KalmanFilter]: https://github.com/dringakn/CarND-Extended-Kalman-Filter-Project/blob/master/src/kalman_filter.cpp
[Tools]: https://github.com/dringakn/CarND-Extended-Kalman-Filter-Project/blob/master/src/tools.cpp  

[//]: # (Image References)

[image1]: ./examples/Result.png "Final Result"
[video1]: ./examples/EKF_Dataset_1.gif "Video"
[video2]: ./examples/EKF_Dataset_2.gif "Video"

# Extended Kalman Filter

The goal of this project is to implement a kalman filter, in C++, to estimate the state of a moving object of interest with noisy lidar and radar measurements. The laser measurements provides the x and y position coordinates while the radar measurements provides the range, bearing and cartesian velocity components. The evaluation metrics are the Root Mean Square Error (RMSE) of car position and velocity, which are are shown as follows: 

![Dataset-1][video1] ![Dataset-2][video2]

Blue circles are radar measurements (position markers inferred from radius and angle; the also-supplied radial velocity measurements are not shown).

Green markers are the car's position as estimated by the Kalman filter. It's clear that the Kalman filter does a good job of tracking the car's position with significantly reduced noise.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

After cloning the repository, the main program can be built and run by doing the following from the project top directory.
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.
