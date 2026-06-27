#include "als_ros/extended_kalman_filter_odom.h"

ExtendedKalmanFilterOdom::ExtendedKalmanFilterOdom(std::string mode)
{
  cov_prior_vel = 10;                // covariance of prior position
  cov_prior_yaw_rate = 10;           // covariance of prior velocity
  double R_cov = 10;                   // covariance of measurement position
  double Q_cov = 0.01;                   // covariance of process noise

  double gravity = 9.80665;
  // double gravity = 0.0;

  double yaw_ang = 0.0 / 180.0 * M_PI;
  double pitch_ang = 0.0 / 180.0 * M_PI;
  double roll_ang = 0.0 / 180.0 * M_PI;
  Eigen::Matrix3d roll_rotation, pitch_rotation, yaw_rotation;
  roll_rotation << 1,0,0,
                   0,cos(roll_ang),-sin(roll_ang),
                   0,sin(roll_ang),cos(roll_ang);
  pitch_rotation << cos(pitch_ang),0,sin(pitch_ang),
                    0,1,0,
                    -sin(pitch_ang),0,cos(pitch_ang);
  yaw_rotation << cos(yaw_ang),-sin(yaw_ang),0,
                  sin(yaw_ang),cos(yaw_ang),0,
                  0,0,1;

  SetCovarianceP(cov_prior_vel, cov_prior_yaw_rate);
  SetCovarianceR(R_cov);
  SetCovarianceQ(Q_cov);
  mode_ = mode;
}

void ExtendedKalmanFilterOdom::SetCovarianceP(double velo_noise, double yaw_rate_noise)
{
  P_.setZero();
  P_ << velo_noise, 0,
        0,yaw_rate_noise;
}

void ExtendedKalmanFilterOdom::SetCovarianceQ(double noise_cov)
{
  Q_.setZero();
  Q_ = TypeMatrixQ::Identity() * noise_cov;
}

void ExtendedKalmanFilterOdom::SetCovarianceR(double noise_cov)
{
  R_.setZero();
  if(mode_ == "normal")
  {
    R_ = TypeMatrixR::Identity() * noise_cov;
  }
}
/**
 * @brief Initialize the state of the filter
 *
 * @param input Initial pose
 */
void ExtendedKalmanFilterOdom::Init(double yaw_rate, double velocity)
{
  state_.setZero();
  state_.block<1, 1>(0, 0) << velocity;
  state_.block<1, 1>(1, 0) << yaw_rate;
}
/**
 * @brief State propagation
 *
 * @param input Imu measurement
 * @param dt Delta time
 */
void ExtendedKalmanFilterOdom::Predict(const sensor_msgs::msg::Imu& input, double dt)
{
  sensor_msgs::msg::Imu curr_imu_data = input;

  double delta_t = dt;

  Eigen::Vector3d curr_accel = Eigen::Vector3d(curr_imu_data.linear_acceleration.x, curr_imu_data.linear_acceleration.y, curr_imu_data.linear_acceleration.z);
  Eigen::Vector3d curr_angle_velocity = Eigen::Vector3d(curr_imu_data.angular_velocity.x, curr_imu_data.angular_velocity.y, curr_imu_data.angular_velocity.z);
  UpdateEkfState(delta_t, curr_accel, curr_angle_velocity);
}
/**
 * @brief Update imu state
 *
 * @param dt Delta time
 * @param accel Acceleration
 * @param curr_angle_velocity Angular velocity
 * @param curr_ori Previous orientation
 */
void ExtendedKalmanFilterOdom::UpdateEkfState(const double dt,
                                          const Eigen::Vector3d& accel, const Eigen::Vector3d& curr_angle_velocity)
{
  static double prev_yaw_rate = 0.0;
  F_.setZero();
  F_.block<2, 2>(0, 0) << 1, 0,
                          0, 1;
  B_.setZero();
  B_.block<2, 2>(0, 0) << dt, 0,
                          0, 1;
  u_.setZero();
  u_ << accel(0), curr_angle_velocity(2) - prev_yaw_rate;

  state_ = F_ * state_ + B_ * u_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  prev_yaw_rate = curr_angle_velocity(2);
}
/**
 * @brief State correction
 *
 * @param measurement Absolute position measurement
 */
void ExtendedKalmanFilterOdom::Correction(double velocity, double yaw_rate)
{
  G_.setZero();
  G_ = TypeMatrixG::Identity();

  Z_ << velocity, yaw_rate;
  K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() +  R_).inverse();
  state_ = state_ + K_ * (Z_ - G_ * state_);
  // P_ = (TypeMatrixP::Identity() - K_ * G_) * P_;
  P_ = P_ - K_ * G_* P_ - P_* G_.transpose()*K_.transpose() + K_*(G_*P_*G_.transpose() + R_)*K_.transpose();
}
