#pragma once

#include <iostream>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

// *** NOTATION *** ///
// pp:    Previous pose
// cp:    Current pose
// dp:    Delta pose
// K:     Kinematic paramters of skid-steering robots
// T_A_B: Transformation matrix from A to B
// H_A_B: Jacobian matrix d(A) / d(B)

class FullLinearWheelOdometyFactor : public gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Pose3, gtsam::Pose3> {
public:
  /**
   * @brief Constructor to define the full linear wheel odometry factor
   * @param kinematic_parameters_key  Key for kinematic paramters of skid-steering robots defined by the full linear model (Eq.10 in our paper)
   * @param previous_pose_key             Key for prebious pose with respect to world frame
   * @param current_pose_key             Key for current pose with respect to world frame
   * @param right_delta_angle         Angular displacement of the right wheel as input [rad]
   * @param left_delta_angle          Delta angle of left encorders as input [rad]
   * @param T_Robot_IMU               Transformation matrix from the robot frame to the IMU frame
   * @param noise_model               Measurement noise model
   */
  FullLinearWheelOdometyFactor(
    gtsam::Key kinematic_parameters_key, 
    gtsam::Key previous_pose_key, gtsam::Key current_pose_key,
    double right_delta_angle, double left_delta_angle,
    gtsam::Pose3 T_Robot_IMU,
    const gtsam::noiseModel::Diagonal::shared_ptr& noise_model)
  : gtsam::NoiseModelFactor3<gtsam::Vector6, gtsam::Pose3, gtsam::Pose3>(noise_model, kinematic_parameters_key, previous_pose_key, current_pose_key),
    right_delta_angle_(right_delta_angle), left_delta_angle_(left_delta_angle), T_Robot_IMU_(T_Robot_IMU) {}

  /**
   * @brief Evaluate error.
   * @param kinematic_parameters  Kinematic paramters of skid-steering robots to be calibrated
   * @param previous_pose         Prebious pose with respect to world frame
   * @param current_pose          Current pose with respect to world frame
   * @param H1                    d(log error) / d(kinematic_parameters)
   * @param H2                    d(log error) / d(previous_pose)
   * @param H3                    d(log error) / d(current_pose)
   * @return                      log error
   */
  virtual gtsam::Vector evaluateError(
    const gtsam::Vector6& ssmr_odom_coefficient_vector, 
    const gtsam::Pose3& previous_pose, const gtsam::Pose3& current_pose, 
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2, boost::optional<gtsam::Matrix&> H3) const override {

    // 1. Calculate a measurement function of the full linear wheel odometry with respect to the robot frame (measuremetn function)
    // 1.1 The robot displacement ∆o_ij ∈ se2 is extended into ∆O_ij ∈ se3.
    // 1.2 ∆O_ij ∈ se3 is converted into SE3.
    gtsam::Vector6 measurement_function_in_robot_frame;
    measurement_function_in_robot_frame << 0.0, 0.0, ssmr_odom_coefficient_vector[4]*left_delta_angle_ + ssmr_odom_coefficient_vector[5]*right_delta_angle_,
                                           ssmr_odom_coefficient_vector[0]*left_delta_angle_ + ssmr_odom_coefficient_vector[1]*right_delta_angle_, ssmr_odom_coefficient_vector[2]*left_delta_angle_ + ssmr_odom_coefficient_vector[3]*right_delta_angle_, 0.0;
    gtsam::Pose3 T_Robot_DeltaRobot = gtsam::Pose3::Expmap(measurement_function_in_robot_frame);    

    // 2. Transform T_Robot_DeltaRobot (the robot displacement in the robot frame) into T_IMU_DeltaIMU (the IMU displacement in the IMU frame) 
    // 2.1 Calculate T_IMU_DeltaIMU bsed on the following equation.
      // T_IMU_DeltaIMU = T_IMU_Robot * T_Robot_DeltaRobot * T_Robot_IMU

    // A is temporary variable name, this is eraced by the chain rule.
    gtsam::Matrix H_A_DeltaRobot;
    gtsam::Pose3 T_Robot_DeltaRobot_T_Robot_IMU = T_Robot_DeltaRobot.compose(T_Robot_IMU_, H_A_DeltaRobot);
    
    gtsam::Matrix H_DeltaIMU_A;
    gtsam::Pose3 T_IMU_DeltaIMU = T_Robot_IMU_.between(T_Robot_DeltaRobot_T_Robot_IMU, boost::none, H_DeltaIMU_A);
    gtsam::Matrix H_DeltaIMU_DeltaRobot = H_DeltaIMU_A * H_A_DeltaRobot;


    // 3. Calculate the error of the full linear wheel odometry factor (Eq.13 in our paper)
    // 3.1 Calculate the delta pose between the previous pose and the current pose.

    // d(T_pp_cp) / d(previous_pose)
    gtsam::Matrix66 H_dp_pp;
    // d(T_pp_cp) / d(current_pose)
    gtsam::Matrix66 H_dp_cp;
    // delta pose between previous pose and current pose
    gtsam::Pose3 T_pp_cp = previous_pose.between(current_pose, H_dp_pp, H_dp_cp);

    // 3.2 Calculate the error

    // d(error) / d(T_pp_cp)
    gtsam::Matrix66 H_e_dp;
    // d(error) / d(h)
    gtsam::Matrix66 H_e_DeltaIMU;
    gtsam::Pose3 error = T_pp_cp.between(T_IMU_DeltaIMU, H_e_dp, H_e_DeltaIMU);
    // d(log error) / d(error)
    gtsam::Matrix6 H_loge_e;
    // error with logmap (Eq.13 in our paper)
    gtsam::Vector6 log_error = error.Logmap(error, H_loge_e);


    // 4. Calculate the jacobian matrix for the optimization values
    // 4.1 Calculate the jacobian matrix for the skid-steering robot's kinematic paramter vector

    if(H1) {
      // d(T_Robot_DeltaRobot) / d(J11)
      gtsam::Matrix61 H_deltaRobot_J11;
      H_deltaRobot_J11 << 0.0,                 // rot X
                          0.0,                 // rot Y
                          0.0,                 // rot Z
                          left_delta_angle_,   // trans X
                          0.0,                 // trans Y
                          0.0;                 // trans Z
      // d(T_Robot_DeltaRobot) / d(J12)
      gtsam::Matrix61 H_deltaRobot_J12;
      H_deltaRobot_J12 << 0.0,
                          0.0,
                          0.0,
                          right_delta_angle_,
                          0.0,
                          0.0;
      // d(T_Robot_DeltaRobot) / d(J21)
      gtsam::Matrix61 H_deltaRobot_J21;
      H_deltaRobot_J21 << 0.0,
                          0.0,
                          0.0,
                          0.0,
                          left_delta_angle_,
                          0.0;
      // d(T_Robot_DeltaRobot) / d(J22)
      gtsam::Matrix61 H_deltaRobot_J22;
      H_deltaRobot_J22 << 0.0,
                          0.0,
                          0.0,
                          0.0,
                          right_delta_angle_,
                          0.0;
      // d(T_Robot_DeltaRobot) / d(J31)
      gtsam::Matrix61 H_deltaRobot_J31;
      H_deltaRobot_J31 << 0.0,
                          0.0,
                          left_delta_angle_,
                          0.0,
                          0.0,
                          0.0;
      // d(T_Robot_DeltaRobot) / d(J32)
      gtsam::Matrix61 H_deltaRobot_J32;
      H_deltaRobot_J32 << 0.0,
                          0.0,
                          right_delta_angle_,
                          0.0,
                          0.0,
                          0.0;
      // d(deltaRobot) / d(K)
      gtsam::Matrix66 H_deltaRobot_K;
      H_deltaRobot_K << H_deltaRobot_J11, H_deltaRobot_J12, H_deltaRobot_J21, H_deltaRobot_J22, H_deltaRobot_J31, H_deltaRobot_J32;
      // d(log error) / d(K)
      *H1 = H_loge_e * H_e_DeltaIMU * H_DeltaIMU_DeltaRobot * H_deltaRobot_K;

    }

    // 4.2 Calculate the jacobian matrix for the previous pose
    if(H2) {
      // d(log error) / d(previous_pose)
      *H2 = H_loge_e * H_e_dp * H_dp_pp;

    }

    // 4.3 Calculate the jacobian matrix for the current pose
    if(H3) {
      // d(log error) / d(current_pose)
      *H3 = H_loge_e * H_e_dp * H_dp_cp;

    }

    return log_error;
  }

private:
  double right_delta_angle_, left_delta_angle_;
  gtsam::Pose3 T_Robot_IMU_;
};
