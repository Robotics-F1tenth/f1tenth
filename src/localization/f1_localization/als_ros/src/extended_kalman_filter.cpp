#include "als_ros/extended_kalman_filter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(std::string mode)
{
  cov_prior_posi = 0.1;          // covariance of prior position
  cov_prior_vel = 100;           // covariance of prior velocity
  cov_prior_ori = 0.1;           // covariance of prior orientation
  cov_prior_epsilon = 0.01;      // covariance of prior gyro bias
  cov_prior_delta = 0.01;        // covariance of prior accel bias
  double R_cov = 1;                   // covariance of measurement position
  double Q_cov = 1;                   // covariance of process noise

  double cov_w_gyro = 0.004;            // covariance of IMU noise gyro
  double cov_w_accel = 0.004;           // covariance of IMU noise accel

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

  SetCovarianceP(cov_prior_posi, cov_prior_vel, cov_prior_ori, cov_prior_epsilon, cov_prior_delta);
  SetCovarianceR(R_cov);
  SetCovarianceQ(Q_cov);
  SetCovarianceW(cov_w_gyro, cov_w_accel);

  gt_.setZero();
  gt_.block<3, 1>(INDEX_STATE_VEL, 0) =  roll_rotation * pitch_rotation *  yaw_rotation * Eigen::Vector3d(0, 0, -gravity);
  // gt_.block<3, 1>(INDEX_STATE_VEL, 0) = Eigen::Vector3d(0, 0, -gravity);

  mode_ = mode;
}

void ExtendedKalmanFilter::SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                                          double gyro_noise, double accel_noise)
{
  P_.setZero();
  P_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * posi_noise;
  P_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * velo_noise;
  P_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise;
  P_.block<3, 3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyro_noise;
  P_.block<3, 3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * accel_noise;
}

void ExtendedKalmanFilter::SetCovarianceW(double gyro_w_noise, double accel_w_noise)
{
  W_.setZero();
  W_.block<3, 1>(0, 0) = Eigen::Vector3d(gyro_w_noise, gyro_w_noise, gyro_w_noise);
  W_.block<3, 1>(3, 0) = Eigen::Vector3d(accel_w_noise, accel_w_noise, accel_w_noise);
}

void ExtendedKalmanFilter::SetCovarianceQ(double noise_cov)
{
  double gyro_bias_noise = 1;
  double accel_bias_noise = 1;
  Q_.setZero();
  Q_ = TypeMatrixQ::Identity() * noise_cov;
  // Q_.block<3,3>(0, 0) = Eigen::Matrix3d::Identity() * noise_cov;
  // Q_.block<3,3>(3, 3) =  Eigen::Matrix3d::Identity() * noise_cov;
}

void ExtendedKalmanFilter::SetCovarianceR(double noise_cov)
{
  R_.setZero();
  // if(mode_ == "adjustment")
  // {
  //   R_.block<3,3>(INDEX_STATE_POSI,INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * noise_cov;
  //   adaptive_R_ = R_;
  // }
  if(mode_ == "normal")
  {
    R_ = TypeMatrixR::Identity() * noise_cov;
    //  R_.block<3,3>(INDEX_STATE_POSI,INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * noise_cov
  }
}
/**
 * @brief Initialize the state of the filter
 *
 * @param input Initial pose
 */
void ExtendedKalmanFilter::Init(geometry_msgs::msg::Pose& input, Eigen::Vector3d velocity)
{
  state_.setZero();
  state_.block<3, 1>(INDEX_STATE_POSI, 0) = Eigen::Vector3d(input.position.x, input.position.y, input.position.z);
  state_.block<3, 1>(INDEX_STATE_VEL, 0) = velocity;

  state_(INDEX_STATE_ORI + 0, 0) = input.orientation.w;
  state_(INDEX_STATE_ORI + 1, 0) = input.orientation.x;
  state_(INDEX_STATE_ORI + 2, 0) = input.orientation.y;
  state_(INDEX_STATE_ORI + 3, 0) = input.orientation.z;

  SetCovarianceP(cov_prior_posi, cov_prior_vel, cov_prior_ori, cov_prior_epsilon, cov_prior_delta);
}
/**
 * @brief State propagation
 *
 * @param input Imu measurement
 * @param dt Delta time
 */
void ExtendedKalmanFilter::Predict(const sensor_msgs::msg::Imu& input, double dt)
{
  sensor_msgs::msg::Imu curr_imu_data = input;

  double delta_t = dt;

  Eigen::Vector3d curr_accel = Eigen::Vector3d(curr_imu_data.linear_acceleration.x, curr_imu_data.linear_acceleration.y, curr_imu_data.linear_acceleration.z);
  Eigen::Vector3d curr_angle_velocity = Eigen::Vector3d(curr_imu_data.angular_velocity.x, curr_imu_data.angular_velocity.y, curr_imu_data.angular_velocity.z);
  curr_angle_velocity = curr_angle_velocity;
  Eigen::Quaterniond curr_ori = Eigen::Quaterniond(pose_.block<3, 3>(0, 0));
  curr_ori.normalize();
  UpdateEkfState(delta_t, curr_accel, curr_angle_velocity, curr_ori);
  UpdateState();
}
/**
 * @brief Update imu state
 *
 * @param dt Delta time
 * @param accel Acceleration
 * @param curr_angle_velocity Angular velocity
 * @param curr_ori Previous orientation
 */
void ExtendedKalmanFilter::UpdateEkfState(const double dt,
                                          const Eigen::Vector3d& accel, const Eigen::Vector3d& curr_angle_velocity,
                                          const Eigen::Quaterniond& curr_ori)
{
  F_.setZero();
  F_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity();

  // Eigen::Vector3d accel_ = accel + state_.block<3, 1>(INDEX_STATE_ACC_BIAS, 0);
  // Eigen::Vector3d gyro_ = curr_angle_velocity + state_.block<3, 1>(INDEX_STATE_GYRO_BIAS, 0);
  Eigen::Vector3d accel_ = accel;
  Eigen::Vector3d gyro_ = curr_angle_velocity;
  gyro_ = gyro_ * dt;

  double q0 = curr_ori.w();
  double q1 = curr_ori.x();
  double q2 = curr_ori.y();
  double q3 = curr_ori.z();
  double FVq0 = 2 * Eigen::Vector3d(q0, -q3, q2).transpose() * accel_;
  double FVq1 = 2 * Eigen::Vector3d(q1, q2, q3).transpose() * accel_;
  double FVq2 = 2 * Eigen::Vector3d(-q2, q1, q0).transpose() * accel_;
  double FVq3 = 2 * Eigen::Vector3d(-q3, -q0, q1).transpose() * accel_;
  Eigen::Matrix<double, 3, 4> FVq = (Eigen::Matrix<double, 3, 4>() << FVq0, FVq1, FVq2, FVq3,
                                                                     -FVq3, -FVq2,FVq1, FVq0,
                                                                      FVq2,-FVq3, -FVq0, FVq1).finished();

  F_.block<3, 4>(INDEX_STATE_VEL, INDEX_STATE_ORI) = FVq;
  F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = pose_.block<3, 3>(0, 0);

  Eigen::Vector3d w = gyro_;
  Eigen::Matrix<double, 4, 4> Fqq =
  0.5 * (Eigen::Matrix<double, 4, 4>() << 0, -w.x(), -w.y(), -w.z(),
                                          w.x(), 0, w.z(), -w.y(),
                                          w.y(), -w.z(), 0, w.x(),
                                          w.z(), w.y(), -w.x(), 0).finished();
  F_.block<4, 4>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Fqq;

  Eigen::Matrix<double, 4, 3> Fqkesi = 0.5 * (Eigen::Matrix<double, 4, 3>() << -q1, -q2, -q3,
                                                                                q0, -q3, q2,
                                                                                q3, q0, -q1,
                                                                                -q2, q1, q0).finished();
  F_.block<4, 3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) = Fqkesi;

  B_.setZero();
  B_.block<3, 3>(INDEX_STATE_VEL, 3) = pose_.block<3, 3>(0, 0);
  B_.block<4, 3>(INDEX_STATE_ORI, 0) = Fqkesi;
  B_.block<3, 3>(INDEX_STATE_GYRO_BIAS, 3) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(INDEX_STATE_ACC_BIAS, 3) = Eigen::Matrix3d::Identity();

  TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * dt;
  TypeMatrixB Bk = B_ * dt;

  state_ = Fk * state_ + Bk * W_ + gt_ * dt;
  // P_ = Fk * P_ * Fk.transpose() + Bk * Q_ * Bk.transpose();
  P_ = Fk * P_ * Fk.transpose() + Q_;
}
/**
 * @brief Update position
 *
 */
void ExtendedKalmanFilter::UpdateState()
{
  pose_.block<3, 1>(0, 3) = state_.block<3, 1>(INDEX_STATE_POSI, 0);

  Eigen::Quaterniond q;
  q.w() = state_(INDEX_STATE_ORI + 0, 0);
  q.x() = state_(INDEX_STATE_ORI + 1, 0);
  q.y() = state_(INDEX_STATE_ORI + 2, 0);
  q.z() = state_(INDEX_STATE_ORI + 3, 0);
  q.normalize();
  pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
}
/**
 * @brief State correction
 *
 * @param measurement Absolute position measurement
 */
void ExtendedKalmanFilter::Correction(const geometry_msgs::msg::Pose& measurement)
{
  Eigen::Quaterniond q;
  q.w() = measurement.orientation.w;
  q.x() = measurement.orientation.x;
  q.y() = measurement.orientation.y;
  q.z() = measurement.orientation.z;
  q.normalize();

  Eigen::Matrix3d rot_mat = q.toRotationMatrix();
  Eigen::Vector3d euler = rot_mat.eulerAngles(2, 1, 0);
  Eigen::Vector3d translation_ = Eigen::Vector3d(-0.12, 0, 0.0);

  TypeVectorX innovation = state_;
  C_.setIdentity();
  G_.setZero();
  G_.block<3,3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();
  G_.block<1,1>(INDEX_STATE_VEL, INDEX_STATE_VEL+2) << 1;
  G_.block<4,4>(INDEX_STATE_VEL+1, INDEX_STATE_ORI) = Eigen::Matrix4d::Identity();

  Eigen::Vector3d lidar_measurement = Eigen::Vector3d(measurement.position.x,
                                                      measurement.position.y,
                                                      measurement.position.z);
  Eigen::Vector3d moved_measurement = lidar_measurement + rot_mat * translation_;

  Z_ << measurement.position.x, measurement.position.y, measurement.position.z, 0.0,
        q.w(), q.x(), q.y(), q.z();
  // Z_ << moved_measurement(0), moved_measurement(1), moved_measurement(2), 0.0,
  //       q.w(), q.x(), q.y(), q.z();
  K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() +  R_).inverse();
  state_ = state_ + K_ * (Z_ - G_ * state_);
  // P_ = (TypeMatrixP::Identity() - K_ * G_) * P_;
  P_ = P_ - K_ * G_* P_ - P_* G_.transpose()*K_.transpose() + K_*(G_*P_*G_.transpose() + R_)*K_.transpose();
  UpdateState();
}
