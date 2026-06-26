#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>
#include <mutex>
#include <optional>

namespace ekf_pose
{

// ────────────────────────────────────────────
//  State vector (6DOF):
//    x(0)=x,  x(1)=y,  x(2)=yaw
//    x(3)=vx, x(4)=vy, x(5)=yaw_rate
// ────────────────────────────────────────────
using StateVec   = Eigen::Matrix<double, 6, 1>;
using StateCov   = Eigen::Matrix<double, 6, 6>;
using Matrix6x6  = Eigen::Matrix<double, 6, 6>;

struct ImuMeasurement {
    double timestamp;
    double ax;         // linear accel x (body frame)
    double ay;         // linear accel y (body frame)
    double yaw_rate;   // angular vel z
};

struct OdomMeasurement {
    double timestamp;
    double vx;
    double vy;
    double yaw_rate;
};

struct PoseMeasurement {
    double timestamp;
    double x;
    double y;
    double yaw;
};

class EKFPoseEstimator : public rclcpp::Node
{
public:
    explicit EKFPoseEstimator();

private:
    // ── EKF core ──────────────────────────────
    void ekf_predict(const ImuMeasurement& imu);
    void ekf_update_pose(const PoseMeasurement& pose);
    void ekf_update_odom(const OdomMeasurement& odom);

    // ── Helpers ───────────────────────────────
    static double normalize_angle(double angle);
    void publish_pose(const rclcpp::Time& stamp);
    void initialize_state(const PoseMeasurement& pose);
    bool is_initialized() const { return initialized_; }

    // ── Callbacks ─────────────────────────────
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mcl_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timer_callback();

    // ── State ─────────────────────────────────
    StateVec  x_;          // state estimate
    StateCov  P_;          // state covariance

    // ── Noise params ──────────────────────────
    // Process noise (Q)
    double q_pos_;         // position process noise
    double q_vel_;         // velocity process noise
    double q_yaw_;         // yaw process noise
    double q_yaw_rate_;    // yaw rate process noise

    // Measurement noise (R)
    double r_mcl_pos_;     // mcl pose position noise
    double r_mcl_yaw_;     // mcl pose yaw noise
    double r_odom_vel_;    // odom velocity noise
    double r_odom_yaw_rate_; // odom yaw rate noise

    // ── Buffers ───────────────────────────────
    std::deque<ImuMeasurement>  imu_buffer_;
    std::deque<OdomMeasurement> odom_buffer_;

    static constexpr size_t IMU_BUF_MAX  = 500;
    static constexpr size_t ODOM_BUF_MAX = 50;

    // ── State flags ───────────────────────────
    bool initialized_     = false;
    bool first_mcl_pose_  = true;
    double last_predict_time_ = 0.0;

    // ── Params ────────────────────────────────
    double publish_rate_;
    bool   publish_tf_;
    bool   use_odom_vel_;
    std::string map_frame_;
    std::string base_frame_;
    std::string odom_topic_;
    std::string imu_topic_;
    std::string mcl_pose_topic_;

    // ── ROS interfaces ────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mcl_pose_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr            pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                    odom_pub_;

    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex state_mutex_;

    // mcl_pose 수신 후 초기화 대기용
    std::optional<PoseMeasurement> pending_init_pose_;
};

}  // namespace ekf_pose
