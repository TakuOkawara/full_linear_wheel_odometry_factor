# Introduction
The full linear wheel odometry factor is a constraint depending on not only robot poses but also the kinematic parameters of a skid-steering robot. The kinematic parameters are defined by the full linear model. Therefore, this factor performs online calibration of kinematic models for skid-steering robots in addition to the motion constraint. 
The main contribution of this factor is two-fold.
* Online calibration of kinematic parameters for skid-steering robots
    * Skid-steering robot's wheel odometry depends on directly-nonobservable phenomena or values (e.g., wheel slippage
      and kinematic model errors).
    * This factor can calibrate the above parameters online.
* Reliable motion constraints
    * Wheel odometry is accurately calculated based on the online calibration.
    * This factor makes an odometry estimation (e.g., LiDAR-IMU odometry) more robust to environments where point clouds degenerate (e.g., long corridors and tunnels).

The following video validates that LiDAR-IMU odometry with our full linear wheel odometry factor accomplishes accurate odometry estimation even in long corridors.
[[video](https://www.youtube.com/watch?v=woLl1c5IenE)]
# Prerequisited
* Ubuntu
* [GTSAM](https://github.com/borglab/gtsam/tree/4.2a9)

We tested this code by Ubuntu 22.04

# Usage
The full linear wheel odometry factor is implemented as a header-only file. Therefore, you can use this factor by only including this header file in your code. Please refer to the examples directory for incorporating this factor into your factor graph defined by GTSAM. Specifically, the example file can be executed based on the following commands.
```commandline
cd examples/
cmake .
make
./full_linear_wheel_odometry_factor_example
```

<!-- # Citation
If you use the full linear wheel odometry factor for academic work, please cite the following publication.  -->
