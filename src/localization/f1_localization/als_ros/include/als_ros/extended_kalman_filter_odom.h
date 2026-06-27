// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <geometry_msgs/Pose.h>
// #include <tf/tf.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class ExtendedKalmanFilterOdom {
private:
  static const unsigned int DIM_STATE = 2;
  static const unsigned int DIM_CONTROL = 2;
  static const unsigned int DIM_STATE_NOISE = 0;        // Angular and accel bias
  static const unsigned int DIM_MEASUREMENT = 2;        // Position
  static const unsigned int DIM_MEASUREMENT_NOISE = 2;  // Position noise

  static const unsigned int INDEX_STATE_POSI = 0;        // Position
  static const unsigned int INDEX_STATE_VEL = 3;         // Velocity
  static const unsigned int INDEX_STATE_ORI = 6;         // Orientation
  static const unsigned int INDEX_STATE_GYRO_BIAS = 10;  // Gyro bias
  static const unsigned int INDEX_STATE_ACC_BIAS = 13;   // Accel bias
  static const unsigned int INDEX_MEASUREMENT_POSI = 0;  // Position

  typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorX;
  typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorC;                // state vector
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorZ;          // measurement vector
  typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixF;        // Covariance matrix
  typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> TypeMatrixB;  // Covariance matrix
  typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixQ;
  typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixP;
  typedef typename Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> TypeMatrixK;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE> TypeMatrixH;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> TypeMatrixG;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> TypeMatrixR;

  TypeVectorX state_;  // Current state and error
  TypeVectorC u_;
  TypeVectorZ Z_;
  TypeMatrixF F_;
  TypeMatrixF B_;
  TypeMatrixQ Q_;
  TypeMatrixP P_;
  TypeMatrixK K_;
  TypeMatrixH C_;
  TypeMatrixG G_;
  TypeMatrixH R_;

  TypeMatrixF Ft_;
  TypeVectorX gt_; // Acceleration term

  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();

  Eigen::Vector3d g_; // gravity

  Eigen::Matrix<double, 3, 3> adaptive_R_;
  std::string mode_;

  double cov_prior_vel;
  double cov_prior_yaw_rate;

  void SetCovarianceP(double velo_noise, double yaw_rate_noise);
public:
  ExtendedKalmanFilterOdom(std::string mode);
  void SetCovarianceQ(double noise_cov);
  void SetCovarianceR(double noise_cov);
  /**
   * @brief Initialize the state of the filter
   *
   * @param input Initial pose
   */
  void Init(double yaw_rate, double velocity);
  /**
   * @brief State propagation
   *
   * @param input Imu measurement
   * @param dt Delta time
   */
  void Predict(const sensor_msgs::msg::Imu& input, double dt);
  /**
   * @brief Update imu state
   *
   * @param dt Delta time
   * @param accel Acceleration
   * @param curr_angle_velocity Angular velocity
   * @param curr_ori Previous orientation
   */
  void UpdateEkfState(const double dt,
                      const Eigen::Vector3d& accel, const Eigen::Vector3d& curr_angle_velocity);
  /**
   * @brief Update position
   *
   */
  void UpdateState();
  /**
   * @brief State correction
   *
   * @param measurement Absolute position measurement
   */
  void Correction(double velocity, double yaw_rate);
  /**
   * @brief Get the State object
   *
   * @return const VectorXd& State vector
   */
  VectorXd getState() { return state_; }
  MatrixXd getCovariance() { return P_; }
};

