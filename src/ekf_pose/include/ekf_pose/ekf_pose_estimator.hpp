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

// retrodiction 용 과거 상태 체크포인트 — predict 스텝마다 하나씩 쌓는다.
struct StateSnapshot {
    double t;
    Eigen::Matrix<double, 6, 1> x;
    Eigen::Matrix<double, 6, 6> P;
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
    void trim_retrodiction_history(double now_t);

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
    // yaw_rate 상태를 IMU 로 직접 대입한 직후 P(5,5)/교차항에 쓰는 값.
    // predict 가 매 스텝 x_(5) 를 새 독립측정치로 덮어쓰므로, 그 시점의
    // "확신도"는 F*P*F'+Q 로 전파된 값이 아니라 자이로 자체의 측정노이즈여야
    // 정합적이다. docs/ekf-pose-analysis-2026-08.md §7.1/§9 P2 참고.
    double r_gyro_yaw_rate_;

    // ── Buffers ───────────────────────────────
    std::deque<ImuMeasurement>  imu_buffer_;   // 초기화 전 리플레이용 (기존)
    std::deque<OdomMeasurement> odom_buffer_;

    static constexpr size_t IMU_BUF_MAX  = 500;
    static constexpr size_t ODOM_BUF_MAX = 50;

    // ── retrodiction 용 롤링 히스토리 (§7.3/P3) ────────────
    // imu_hist_: 실제 IMU 측정 로그 (시간 기반 트림, 절대 재작성 안 됨).
    // state_hist_: 그 IMU를 순서대로 predict 한 결과 스냅샷 — retrodiction
    // 이후엔 보정된 궤적으로 다시 채워진다(imu_hist_에서 재계산).
    std::deque<ImuMeasurement>  imu_hist_;
    std::deque<StateSnapshot>   state_hist_;
    bool   retrodiction_enabled_ = true;
    double retrodiction_window_  = 0.5;   // [s] 보관 구간 = retrodiction 가능한 최대 지연

    // ── State flags ───────────────────────────
    bool initialized_     = false;
    bool first_mcl_pose_  = true;
    double last_predict_time_ = 0.0;
    // 마지막으로 상태가 유효했던 시각(마지막 predict 또는 초기화 시각).
    // publish_pose 가 this->now() 대신 이걸 써야 발행 타임스탬프가 상태의
    // 실제 유효시각과 일치한다 (docs/ekf-pose-analysis-2026-08.md §7.4/P3).
    rclcpp::Time last_state_stamp_{0, 0, RCL_ROS_TIME};

    // ── Params ────────────────────────────────
    double publish_rate_;
    bool   publish_tf_;
    bool   use_odom_vel_;
    // IMU 가 차체 대비 yaw 180도 회전 장착 → ax,ay 부호 반전 보정 (자이로 z 는 정상).
    // ⚠ predict 의 Coriolis 부호와 반드시 세트로 다룰 것. docs/ekf-pose-analysis-2026-08.md
    bool   imu_axis_yaw180_ = true;
    // /odom 의 angular.z(운동학 재구성)를 yaw_rate 측정치로 융합할지.
    // 실측상 코너에서 실제의 0.7~0.8배 + 115ms 지연이라 기본값 false.
    bool   use_odom_yaw_rate_ = false;
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
