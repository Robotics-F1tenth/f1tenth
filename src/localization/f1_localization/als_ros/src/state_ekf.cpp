#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <numeric>

// ROS2 specific headers
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Message types
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "f1_msgs/msg/car_state.hpp"

using namespace std::chrono_literals;
using Eigen::Matrix2d;
using Eigen::Vector2d;

class StateEKF : public rclcpp::Node
{
public:
  StateEKF()
      : Node("state_ekf_node"),
        is_initialized_(false),
        is_odom_received_(false),
        is_imu_received_(false),
        is_pose_received_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("filter_mode", "MAF");
    this->get_parameter("filter_mode").as_string();
    this->declare_parameter<int>("window_size", 5);
    this->declare_parameter<double>("dt", 0.02);
    this->declare_parameter<double>("lambda", 0.5);
    this->declare_parameter<std::vector<double>>("process_noise_covariance", {0.01, 0, 0, 0.01});
    this->declare_parameter<std::vector<double>>("measurement_noise_covariance", {0.5, 0, 0, 0.5});

    // Get parameters
    std::string filter_mode_str = this->get_parameter("filter_mode").as_string();
    if (filter_mode_str == "LPF")
    {
      filter_mode_ = LPF;
    }
    else if (filter_mode_str == "MAF")
    {
      filter_mode_ = MAF;
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Invalid filter_mode parameter. Defaulting to LPF.");
      filter_mode_ = LPF;
    }

    window_size_ = this->get_parameter("window_size").as_int();
    dt_ = this->get_parameter("dt").as_double();
    lambda_ = this->get_parameter("lambda").as_double();

    // Get process noise covariance matrix
    std::vector<double> Q_values = this->get_parameter("process_noise_covariance").as_double_array();
    if (Q_values.size() == 4)
    {
      Q_ << Q_values[0], Q_values[1], Q_values[2], Q_values[3];
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Invalid process_noise_covariance parameter. Using default.");
      Q_ = Matrix2d::Identity() * 0.01;
    }

    // Get measurement noise covariance matrix
    std::vector<double> R_values = this->get_parameter("measurement_noise_covariance").as_double_array();
    if (R_values.size() == 4)
    {
      R_ << R_values[0], R_values[1], R_values[2], R_values[3];
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Invalid measurement_noise_covariance parameter. Using default.");
      R_ = Matrix2d::Identity() * 0.5;
    }

    // Initialize state covariance matrix
    P_ = Matrix2d::Identity();

    // Initialize subscribers
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "mcl_pose", rclcpp::QoS(1).reliable(),
        std::bind(&StateEKF::pose_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::QoS(1).reliable(),
        std::bind(&StateEKF::imu_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(1).reliable(),
        std::bind(&StateEKF::odom_callback, this, std::placeholders::_1));

    control_input_sub_ = create_subscription<f1_msgs::msg::CarState>(
        "kf_state0", rclcpp::QoS(1).reliable(),
        std::bind(&StateEKF::control_input_callback, this, std::placeholders::_1));

    // Initialize publishers
    state_pub_ = create_publisher<f1_msgs::msg::CarState>(
        "/filtered_state", rclcpp::QoS(1).reliable());

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/filtered_pose", rclcpp::QoS(1).reliable());

    // Initialize timer for main loop
    main_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(20ms),
        std::bind(&StateEKF::main_loop, this));
  }

private:
  // ROS2 components
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<f1_msgs::msg::CarState>::SharedPtr control_input_sub_;
  rclcpp::Publisher<f1_msgs::msg::CarState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // EKF state variables
  Vector2d x_; // State vector [vx, vy]
  Matrix2d P_; // Covariance matrix
  Matrix2d Q_; // Process noise covariance
  Matrix2d R_; // Measurement noise covariance

  // Filter modes
  enum FilterMode
  {
    LPF,
    MAF
  };
  FilterMode filter_mode_;
  int window_size_;

  // Flags
  bool is_initialized_;
  bool is_odom_received_;
  bool is_imu_received_;
  bool is_pose_received_;

  // Time step and smoothing factor
  double dt_;     // Time step
  double lambda_; // Smoothing factor for low-pass filter

  // Struct for raw sensor data
  struct SensorData
  {
    double ax = 0.0;
    double ay = 0.0;
    double omega = 0.0;
    double posX = 0.0;
    double posY = 0.0;
    double yaw = 0.0;
  } sensor_data_;

  // Struct for filtered data
  struct FilteredData
  {
    double ax = 0.0;
    double ay = 0.0;
    double omega = 0.0;
    double posX = 0.0;
    double posY = 0.0;
    double slip_angle = 0.0;
  };

  FilteredData filtered_data_;
  FilteredData prev_filtered_data_;

  // Struct for filter buffers (used in MAF)
  struct FilterBuffers
  {
    std::deque<double> posX;
    std::deque<double> posY;
    std::deque<double> ax;
    std::deque<double> ay;
    std::deque<double> omega;
    std::deque<double> slip_angle;
  } filter_buffers_;

  // Other variables
  double v_scale_ = 0.0;
  double scaled_vx_ = 0.0, scaled_vy_ = 0.0;
  double accel_ = 0.0, steer_ = 0.0;

  /**
   * @brief Main loop executed at a fixed rate
   */
  void main_loop()
  {
    // Initialize on first run
    if (!is_initialized_)
    {
      initialize();
      return;
    }

    // Apply filter to measurements
    apply_filter();

    // Execute EKF prediction and update steps
    predict();
    update();

    // Post-process the state
    post_processing();

    // Publish the filtered state and pose
    publish_state();
    publish_pose();

    // Update previous filtered measurements
    prev_filtered_data_ = filtered_data_;
  }

  /**
   * @brief Initializes the EKF with initial measurements
   */
  void initialize()
  {
    if (!is_imu_received_)
      RCLCPP_INFO(get_logger(), "Waiting for IMU data...");
    if (!is_odom_received_)
      RCLCPP_INFO(get_logger(), "Waiting for Odometry data...");
    if (!is_pose_received_)
      RCLCPP_INFO(get_logger(), "Waiting for Pose data...");

    if (is_imu_received_ && is_odom_received_ && is_pose_received_)
    {
      // Initialize filtered measurements with the first measurements
      filtered_data_.posX = sensor_data_.posX;
      filtered_data_.posY = sensor_data_.posY;
      filtered_data_.ax = sensor_data_.ax;
      filtered_data_.ay = sensor_data_.ay;
      filtered_data_.omega = sensor_data_.omega;

      // Set previous filtered measurements
      prev_filtered_data_ = filtered_data_;

      // Initialize buffers for MAF
      if (filter_mode_ == MAF)
      {
        filter_buffers_.posX.assign(window_size_, filtered_data_.posX);
        filter_buffers_.posY.assign(window_size_, filtered_data_.posY);
        filter_buffers_.ax.assign(window_size_, filtered_data_.ax);
        filter_buffers_.ay.assign(window_size_, filtered_data_.ay);
        filter_buffers_.omega.assign(window_size_, filtered_data_.omega);
        filter_buffers_.slip_angle.assign(window_size_, 0.0);
      }

      is_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Initialization complete.");
    }
  }

  /**
   * @brief Applies a filter (LPF or MAF) to the measurements
   */
  void apply_filter()
  {
    if (filter_mode_ == LPF)
    {
      // Position
      filtered_data_.posX = lambda_ * prev_filtered_data_.posX + (1 - lambda_) * sensor_data_.posX;
      filtered_data_.posY = lambda_ * prev_filtered_data_.posY + (1 - lambda_) * sensor_data_.posY;

      // Acceleration and angular velocity
      filtered_data_.ax = lambda_ * prev_filtered_data_.ax + (1 - lambda_) * sensor_data_.ax;
      filtered_data_.ay = lambda_ * prev_filtered_data_.ay + (1 - lambda_) * sensor_data_.ay;
      filtered_data_.omega = lambda_ * prev_filtered_data_.omega + (1 - lambda_) * sensor_data_.omega;
    }
    else if (filter_mode_ == MAF)
    {
      // Add current measurements to buffers
      filter_buffers_.posX.push_back(sensor_data_.posX);
      filter_buffers_.posY.push_back(sensor_data_.posY);
      filter_buffers_.ax.push_back(sensor_data_.ax);
      filter_buffers_.ay.push_back(sensor_data_.ay);
      filter_buffers_.omega.push_back(sensor_data_.omega);

      // Keep buffer size within window_size_
      if (filter_buffers_.posX.size() > window_size_)
      {
        filter_buffers_.posX.pop_front();
        filter_buffers_.posY.pop_front();
        filter_buffers_.ax.pop_front();
        filter_buffers_.ay.pop_front();
        filter_buffers_.omega.pop_front();
      }

      // Compute averages
      filtered_data_.posX = std::accumulate(filter_buffers_.posX.begin(), filter_buffers_.posX.end(), 0.0) / filter_buffers_.posX.size();
      filtered_data_.posY = std::accumulate(filter_buffers_.posY.begin(), filter_buffers_.posY.end(), 0.0) / filter_buffers_.posY.size();
      filtered_data_.ax = std::accumulate(filter_buffers_.ax.begin(), filter_buffers_.ax.end(), 0.0) / filter_buffers_.ax.size();
      filtered_data_.ay = std::accumulate(filter_buffers_.ay.begin(), filter_buffers_.ay.end(), 0.0) / filter_buffers_.ay.size();
      filtered_data_.omega = std::accumulate(filter_buffers_.omega.begin(), filter_buffers_.omega.end(), 0.0) / filter_buffers_.omega.size();
    }
  }

  /**
   * @brief Callback function for IMU data
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    is_imu_received_ = true;
    sensor_data_.ax = msg->linear_acceleration.x;
    sensor_data_.ay = msg->linear_acceleration.y;
    sensor_data_.omega = msg->angular_velocity.z;
  }

  /**
   * @brief Callback function for Odometry data
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    is_odom_received_ = true;
    v_scale_ = msg->twist.twist.linear.x;
  }

  /**
   * @brief Callback function for Pose data
   */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    is_pose_received_ = true;
    sensor_data_.posX = msg->pose.position.x;
    sensor_data_.posY = msg->pose.position.y;

    // Extract yaw from quaternion
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, sensor_data_.yaw);
  }

  /**
   * @brief Callback function for control input data
   */
  void control_input_callback(const f1_msgs::msg::CarState::SharedPtr msg)
  {
    accel_ = msg->accel;
    steer_ = msg->steer;
  }

  /**
   * @brief EKF prediction step
   */
  void predict()
  {
    // State prediction using motion model
    Vector2d x_pred;
    x_pred(0) = x_(0) + dt_ * (filtered_data_.ax + x_(1) * filtered_data_.omega);
    x_pred(1) = x_(1) + dt_ * (filtered_data_.ay - x_(0) * filtered_data_.omega);

    // Jacobian matrix of the motion model
    Matrix2d A = Matrix2d::Identity();
    A(0, 1) = dt_ * filtered_data_.omega;
    A(1, 0) = -dt_ * filtered_data_.omega;

    // Update process covariance
    P_ = A * P_ * A.transpose() + Q_;

    // Update state
    x_ = x_pred;
  }

  /**
   * @brief EKF update step
   */
  void update()
  {
    // Compute change in position
    double delta_x = filtered_data_.posX - prev_filtered_data_.posX;
    double delta_y = filtered_data_.posY - prev_filtered_data_.posY;

    // Convert position differences to velocities in the body frame
    double vx_meas = (delta_x * std::cos(sensor_data_.yaw) + delta_y * std::sin(sensor_data_.yaw)) / dt_;
    double vy_meas = (delta_y * std::cos(sensor_data_.yaw) - delta_x * std::sin(sensor_data_.yaw)) / dt_;

    Vector2d z;
    z << vx_meas, vy_meas;

    // Measurement matrix
    Matrix2d H = Matrix2d::Identity();

    // Measurement residual
    Vector2d y = z - H * x_;

    // Residual covariance
    Matrix2d S = H * P_ * H.transpose() + R_;

    // Kalman gain
    Matrix2d K = P_ * H.transpose() * S.inverse();

    // Update state estimate
    x_ = x_ + K * y;

    // Update covariance estimate
    P_ = (Matrix2d::Identity() - K * H) * P_;
  }

  /**
   * @brief Post-processing step to adjust velocities
   */
  void post_processing()
  {
    // Compute slip angle
    double slip_angle = std::atan2(x_(1), x_(0));

    if (filter_mode_ == LPF)
    {
      filtered_data_.slip_angle = lambda_ * prev_filtered_data_.slip_angle + (1 - lambda_) * slip_angle;
    }
    else if (filter_mode_ == MAF)
    {
      filter_buffers_.slip_angle.push_back(slip_angle);
      if (filter_buffers_.slip_angle.size() > window_size_)
      {
        filter_buffers_.slip_angle.pop_front();
      }
      filtered_data_.slip_angle = std::accumulate(filter_buffers_.slip_angle.begin(), filter_buffers_.slip_angle.end(), 0.0) / filter_buffers_.slip_angle.size();
    }

    // Scale velocities based on odometry speed
    scaled_vx_ = v_scale_ * std::cos(filtered_data_.slip_angle);
    scaled_vy_ = v_scale_ * std::sin(filtered_data_.slip_angle);
  }

  /**
   * @brief Publishes the filtered state
   */
  void publish_state()
  {
    f1_msgs::msg::CarState filtered_state;
    filtered_state.px = filtered_data_.posX;
    filtered_state.py = filtered_data_.posY;
    filtered_state.yaw = sensor_data_.yaw;
    filtered_state.v = x_.norm();
    filtered_state.slip_angle = filtered_data_.slip_angle;
    filtered_state.vx = scaled_vx_;
    filtered_state.vy = scaled_vy_;
    // filtered_state.vx = x_(0);
    // filtered_state.vy = x_(1);
    filtered_state.ax = filtered_data_.ax;
    filtered_state.ay = filtered_data_.ay;
    filtered_state.omega = filtered_data_.omega;
    filtered_state.accel = accel_;
    filtered_state.steer = steer_;

    state_pub_->publish(filtered_state);
  }

  /**
   * @brief Publishes the filtered pose
   */
  void publish_pose()
  {
    geometry_msgs::msg::PoseStamped filtered_pose;
    filtered_pose.header.stamp = now();
    filtered_pose.header.frame_id = "map";
    filtered_pose.pose.position.x = filtered_data_.posX;
    filtered_pose.pose.position.y = filtered_data_.posY;
    filtered_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, sensor_data_.yaw);
    q.normalize();
    filtered_pose.pose.orientation = tf2::toMsg(q);

    pose_pub_->publish(filtered_pose);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateEKF>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
