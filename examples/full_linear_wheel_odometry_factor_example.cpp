#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

// **** Please include this header file to use the full linear wheel odometry factor. **** //
#include "../full_linear_wheel_odometry_factor.hpp"

using gtsam::symbol_shorthand::X;  // IMU pose
using gtsam::symbol_shorthand::K;  // kinematic parameters of skid-steering robots expressed by the full linear model

///////////////////////////////////////////////////////////
///////////////////////  Overview  ////////////////////////
///////////////////////////////////////////////////////////
// This example shows how to use the full linear wheel odometry factor.
// This example has only straight motion, thus J11 and J22 are calibrated such that J11:1.05 -> 1, J12:1.05 -> 1.
// This example is created by extending OdometryExample.cpp, GTSAM example.

// Convert gtsam::Pose2 to gtsam::Pose3
gtsam::Pose3 gtsamPose2ToPose3(const gtsam::Pose2 pose2)
{
  gtsam::Pose3 pose3(gtsam::Rot3::AxisAngle(gtsam::Point3(0.0, 0.0, 1.0), pose2.theta()), 
                     gtsam::Point3(pose2.x(), pose2.y(), 0.0));
  return pose3;
}

int main(int argc, char** argv) {
  // Create an empty nonlinear factor graph
  gtsam::NonlinearFactorGraph graph;

  ///////////////////////////////////////////////////////////
  /////  Full linear wheel odometry factor constraints  /////
  ///////////////////////////////////////////////////////////

  // Prior factors and initial values
  double wheel_radius = 2.1;  // [m]  Please use the actual wheel radius of your robot.
  double wheel_base = 1.0;    // [m]  Please use the actual wheelbase of your robot.

  // Initial value is set by using the ideal-differential drive model.
  gtsam::Vector6 initial_K;
  initial_K << 0.5*wheel_radius, 0.5*wheel_radius, 0.0, 0.0, -wheel_radius/wheel_base, wheel_radius/wheel_base;
  graph.addPrior(K(1), initial_K);

  // Simulate a displacement of wheels for straightt movement. Please use actual values of your data.
  double right_wheels_delta_angle_for_straight_movement = 1.0; // [rad]
  double left_wheel_delta_angle_for_straight_movement = 1.0; // [rad]

  // Transformation between the robot frame and IMU frame
  gtsam::Pose3 T_Robot_Imu = gtsam::Pose3();

  // The uncertainty of full linear model (In this sample code, a constant matrix is used.)
  auto wheel_odom_noisemodel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3));

  // Create the full linear wheel odometry factors
  graph.add(FullLinearWheelOdometyFactor(K(1),
                                         X(1), X(2),
                                         right_wheels_delta_angle_for_straight_movement, left_wheel_delta_angle_for_straight_movement,
                                         T_Robot_Imu,
                                         wheel_odom_noisemodel));
  graph.add(FullLinearWheelOdometyFactor(K(2),
                                         X(2), X(3),
                                         right_wheels_delta_angle_for_straight_movement, left_wheel_delta_angle_for_straight_movement,
                                         T_Robot_Imu,
                                         wheel_odom_noisemodel));

  // Create the kinematic parameter fixation factor
  gtsam::Vector6 zero_mean_constraints;
  zero_mean_constraints.setZero();
  auto random_walk_noisemodel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector6> >(K(1), K(2), zero_mean_constraints, random_walk_noisemodel);

  // Set initial values
  // For illustrative purposes, these have been deliberately set to incorrect values
  // K = [J11 J21 J21 J22 J31 J32]^T, J elements are based on full linear model.
  gtsam::Values initial;
  initial.insert(K(1), initial_K);
  initial.insert(K(2), initial_K);


  ///////////////////////////////////////////////////////////
  ///////////////////  Other constraints  ///////////////////
  ///////////////////////////////////////////////////////////

  // Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  gtsam::Pose2 priorMean(0.0, 0.0, 0.0);  // prior at origin
  gtsam::Pose3 priorMean3D = gtsamPose2ToPose3(priorMean);
  auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4));
  graph.addPrior(X(1), priorMean3D, priorNoise);

  // These constraints are equivalent to Matching cost factor and IMU preintegration factor.
  gtsam::Pose2 odometry(2.0, 0.0, 0.0);
  gtsam::Pose3 odometry3D = gtsamPose2ToPose3(odometry);
  // For simplicity, we will use the same noise model for each odometry factor
  auto odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4));
  // Create odometry (Between) factors between consecutive poses
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(X(1), X(2), odometry3D, odometryNoise);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(X(2), X(3), odometry3D, odometryNoise);


  initial.insert(X(1), gtsamPose2ToPose3(gtsam::Pose2(0.5, 0.0, 0.2)));
  initial.insert(X(2), gtsamPose2ToPose3(gtsam::Pose2(2.3, 0.1, -0.2)));
  initial.insert(X(3), gtsamPose2ToPose3(gtsam::Pose2(4.1, 0.1, 0.1)));
  initial.print("\n*** Initial Estimate: ***\n");  // print

  // optimize using Levenberg-Marquardt optimization
  // K = [J11 J21 J21 J22 J31 J32]^T, J elements are based on full linear model.
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("*** Final Result: ***\n");

  std::cout << std::endl << std::endl;
  std::cout << "##### initial kinematic parameters of skid-steering robots #####" << std::endl;
  for(int i=1; i<3; i++)
  {
    std::cout << "K" << i << std::endl << initial.at<gtsam::Vector6>(K(i)) << std::endl;
  }

  std::cout << "##### calibrated kinematic parameters of skid-steering robots #####" << std::endl;
  for(int i=1; i<3; i++)
  {
    std::cout << "K" << i << std::endl << result.at<gtsam::Vector6>(K(i)) << std::endl;
  }

  std::cout << "\nYou can see J11 and J12 of the full linear model were changed.\n";
  std::cout << "K = [J11 J21 J21 J22 J31 J32]^T, J elements are based on the full linear model.\n";
  

  return 0;
}