# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Dataset #1 Output

```
Accuracy - RMSE:
0.0651649
0.0605378
  0.54319
 0.544191

```

![Dataset #1 Output](https://github.com/mleonardallen/CarND-Extended-Kalman-Filter-Project/blob/master/images/data-1.png)


## Dataset #2 Output

```
Accuracy - RMSE:
0.185496
0.190302
0.476755
0.804468
```

![Dataset #2 Output](https://github.com/mleonardallen/CarND-Extended-Kalman-Filter-Project/blob/master/images/data-2.png)
