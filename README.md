[MainCode]: https://github.com/dringakn/CarND-Extended-Kalman-Filter-Project/blob/master/src/main.cpp
[FusionEKF]: https://github.com/dringakn/CarND-Extended-Kalman-Filter-Project/blob/master/src/FusionEKF.cpp
[KalmanFilter]: https://github.com/dringakn/CarND-Extended-Kalman-Filter-Project/blob/master/src/kalman_filter.cpp
[Tools]: https://github.com/dringakn/CarND-Extended-Kalman-Filter-Project/blob/master/src/tools.cpp  

[//]: # (Image References)

[image1]: ./examples/Convergance_Result.png "Final Result"
[video1]: ./examples/Particle_Filter_For_Robot_Kidnap_Problem.gif "Video"

# Extended Kalman Filter Project Starter Code

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

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

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Reflection
