#include "ekf_pose/ekf_pose_estimator.hpp"
#include <tf2/utils.h>
#include <cmath>

namespace ekf_pose
{

// ════════════════════════════════════════════════════════════════
//  Constructor – 파라미터 선언 / 구독·발행 설정
// ════════════════════════════════════════════════════════════════
EKFPoseEstimator::EKFPoseEstimator() : Node("ekf_pose_estimator")
{
    // ── 파라미터 선언 ─────────────────────────────────────────
    declare_parameter("publish_rate",       200.0);
    declare_parameter("publish_tf",         true);
    declare_parameter("use_odom_vel",       true);
    declare_parameter("map_frame",          "map");
    declare_parameter("base_frame",         "base_link");
    declare_parameter("odom_topic",         "/odom");
    declare_parameter("imu_topic",          "/imu/data");
    declare_parameter("mcl_pose_topic",     "/newmcl_pose");

    // Process noise
    declare_parameter("q_pos",              0.01);
    declare_parameter("q_vel",              0.5);
    declare_parameter("q_yaw",              0.01);
    declare_parameter("q_yaw_rate",         0.3);

    // Measurement noise
    declare_parameter("r_mcl_pos",          0.05);
    declare_parameter("r_mcl_yaw",          0.02);
    declare_parameter("r_odom_vel",         0.1);
    declare_parameter("r_odom_yaw_rate",    0.05);

    // ── 파라미터 읽기 ─────────────────────────────────────────
    publish_rate_      = get_parameter("publish_rate").as_double();
    publish_tf_        = get_parameter("publish_tf").as_bool();
    use_odom_vel_      = get_parameter("use_odom_vel").as_bool();
    map_frame_         = get_parameter("map_frame").as_string();
    base_frame_        = get_parameter("base_frame").as_string();
    odom_topic_        = get_parameter("odom_topic").as_string();
    imu_topic_         = get_parameter("imu_topic").as_string();
    mcl_pose_topic_    = get_parameter("mcl_pose_topic").as_string();

    q_pos_             = get_parameter("q_pos").as_double();
    q_vel_             = get_parameter("q_vel").as_double();
    q_yaw_             = get_parameter("q_yaw").as_double();
    q_yaw_rate_        = get_parameter("q_yaw_rate").as_double();

    r_mcl_pos_         = get_parameter("r_mcl_pos").as_double();
    r_mcl_yaw_         = get_parameter("r_mcl_yaw").as_double();
    r_odom_vel_        = get_parameter("r_odom_vel").as_double();
    r_odom_yaw_rate_   = get_parameter("r_odom_yaw_rate").as_double();

    // ── 상태 초기화 ───────────────────────────────────────────
    x_.setZero();
    P_.setIdentity();
    P_ *= 1.0;   // 초기 불확실성

    // ── 구독 ──────────────────────────────────────────────────
    auto imu_qos = rclcpp::SensorDataQoS();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos,
        std::bind(&EKFPoseEstimator::imu_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&EKFPoseEstimator::odom_callback, this, std::placeholders::_1));

    mcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        mcl_pose_topic_, 10,
        std::bind(&EKFPoseEstimator::mcl_pose_callback, this, std::placeholders::_1));

    // ── 발행 ──────────────────────────────────────────────────
    pose_pub_     = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/ekf_pose", 10);

    pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ekf_pose_with_covariance", 10);

    odom_pub_     = create_publisher<nav_msgs::msg::Odometry>(
        "/ekf_odom", 10);

    // ── TF 브로드캐스터 ───────────────────────────────────────
    if (publish_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    // ── 발행 타이머 (publish_rate Hz) ─────────────────────────
    auto period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / publish_rate_));
    pub_timer_ = create_wall_timer(
        period_ms, std::bind(&EKFPoseEstimator::timer_callback, this));

    RCLCPP_INFO(get_logger(),
        "[EKF Pose] Started — IMU: %s | Odom: %s | MCL: %s | Rate: %.0f Hz",
        imu_topic_.c_str(), odom_topic_.c_str(), mcl_pose_topic_.c_str(), publish_rate_);
}

// ════════════════════════════════════════════════════════════════
//  헬퍼: 각도 정규화 [-π, π]
// ════════════════════════════════════════════════════════════════
double EKFPoseEstimator::normalize_angle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ════════════════════════════════════════════════════════════════
//  초기화: mcl_pose 첫 수신 시 상태를 세팅
// ════════════════════════════════════════════════════════════════
void EKFPoseEstimator::initialize_state(const PoseMeasurement& pose)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    x_.setZero();
    x_(0) = pose.x;
    x_(1) = pose.y;
    x_(2) = pose.yaw;
    // vx, vy, yaw_rate 는 0으로 시작

    P_.setIdentity();
    P_(0, 0) = r_mcl_pos_;
    P_(1, 1) = r_mcl_pos_;
    P_(2, 2) = r_mcl_yaw_;
    P_(3, 3) = 1.0;
    P_(4, 4) = 1.0;
    P_(5, 5) = 0.5;

    last_predict_time_ = pose.timestamp;
    initialized_ = true;

    RCLCPP_INFO(get_logger(),
        "[EKF Pose] Initialized at (%.3f, %.3f, yaw=%.3f)",
        pose.x, pose.y, pose.yaw);
}

// ════════════════════════════════════════════════════════════════
//  EKF Predict Step  —  IMU 기반 비선형 동역학 모델
//
//  x(k+1) = f(x(k), u_imu)
//
//  도출 모델 (자전거 모델 + IMU 가속도):
//    px  += (vx*cos(yaw) - vy*sin(yaw)) * dt
//    py  += (vx*sin(yaw) + vy*cos(yaw)) * dt
//    yaw += yaw_rate * dt
//    vx  += (ax - vy*yaw_rate) * dt          ← 코리올리 보정
//    vy  += (ay + vx*yaw_rate) * dt
//    yaw_rate = IMU measurement (직접 대입)
// ════════════════════════════════════════════════════════════════
void EKFPoseEstimator::ekf_predict(const ImuMeasurement& imu)
{
    if (!initialized_) return;

    double dt = imu.timestamp - last_predict_time_;
    if (dt <= 0.0 || dt > 0.5) {
        last_predict_time_ = imu.timestamp;
        return;
    }
    last_predict_time_ = imu.timestamp;

    // 현재 상태 추출
    double px       = x_(0);
    double py       = x_(1);
    double yaw      = x_(2);
    double vx       = x_(3);
    double vy       = x_(4);
    // yaw_rate 는 IMU에서 직접 받아 사용
    double yr       = imu.yaw_rate;

    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    // ── 비선형 상태 전이 f(x) ─────────────────────────────
    double px_new   = px + (vx * cy - vy * sy) * dt;
    double py_new   = py + (vx * sy + vy * cy) * dt;
    double yaw_new  = normalize_angle(yaw + yr * dt);
    double vx_new   = vx + (imu.ax - vy * yr) * dt;
    double vy_new   = vy + (imu.ay + vx * yr) * dt;
    double yr_new   = yr;   // IMU를 신뢰

    x_(0) = px_new;
    x_(1) = py_new;
    x_(2) = yaw_new;
    x_(3) = vx_new;
    x_(4) = vy_new;
    x_(5) = yr_new;

    // ── 야코비안 F = ∂f/∂x ────────────────────────────────
    //   (수치 미분 없이 해석적으로 도출)
    Matrix6x6 F;
    F.setIdentity();

    //  ∂px/∂yaw = (-vx*sin(yaw) - vy*cos(yaw)) * dt
    F(0, 2) = (-vx * sy - vy * cy) * dt;
    //  ∂px/∂vx = cos(yaw)*dt
    F(0, 3) = cy * dt;
    //  ∂px/∂vy = -sin(yaw)*dt
    F(0, 4) = -sy * dt;

    //  ∂py/∂yaw = (vx*cos(yaw) - vy*sin(yaw)) * dt
    F(1, 2) = (vx * cy - vy * sy) * dt;
    //  ∂py/∂vx = sin(yaw)*dt
    F(1, 3) = sy * dt;
    //  ∂py/∂vy = cos(yaw)*dt
    F(1, 4) = cy * dt;

    //  ∂yaw/∂yaw_rate = dt  (yaw_rate 를 상태로 포함)
    F(2, 5) = dt;

    //  ∂vx/∂vy = -yr*dt
    F(3, 4) = -yr * dt;
    //  ∂vy/∂vx = yr*dt
    F(4, 3) =  yr * dt;

    // ── 프로세스 노이즈 Q ──────────────────────────────────
    Matrix6x6 Q;
    Q.setZero();
    Q(0, 0) = q_pos_      * dt * dt;
    Q(1, 1) = q_pos_      * dt * dt;
    Q(2, 2) = q_yaw_      * dt * dt;
    Q(3, 3) = q_vel_      * dt;
    Q(4, 4) = q_vel_      * dt;
    Q(5, 5) = q_yaw_rate_ * dt;

    // ── 공분산 전파 P = F*P*F' + Q ────────────────────────
    P_ = F * P_ * F.transpose() + Q;
}

// ════════════════════════════════════════════════════════════════
//  EKF Update Step — MCL Pose (x, y, yaw)
//
//  관측 모델: z = H*x + noise
//    H = [1 0 0 0 0 0]   (px)
//        [0 1 0 0 0 0]   (py)
//        [0 0 1 0 0 0]   (yaw)
// ════════════════════════════════════════════════════════════════
void EKFPoseEstimator::ekf_update_pose(const PoseMeasurement& pose)
{
    if (!initialized_) return;

    // 관측 행렬 H (3×6)
    Eigen::Matrix<double, 3, 6> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;

    // 측정 노이즈 R (3×3)
    Eigen::Matrix<double, 3, 3> R;
    R.setZero();
    R(0, 0) = r_mcl_pos_;
    R(1, 1) = r_mcl_pos_;
    R(2, 2) = r_mcl_yaw_;

    // 잔차 y = z - H*x
    Eigen::Vector3d z(pose.x, pose.y, pose.yaw);
    Eigen::Vector3d y_res = z - H * x_;
    y_res(2) = normalize_angle(y_res(2));   // yaw wrap

    // 혁신 공분산 S = H*P*H' + R
    Eigen::Matrix<double, 3, 3> S = H * P_ * H.transpose() + R;

    // 칼만 게인 K = P*H'*S^{-1}
    Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    // 상태 업데이트
    x_ = x_ + K * y_res;
    x_(2) = normalize_angle(x_(2));

    // 공분산 업데이트 (Joseph form으로 수치 안정성 확보)
    Matrix6x6 I_KH = Matrix6x6::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

// ════════════════════════════════════════════════════════════════
//  EKF Update Step — Odometry (vx, vy, yaw_rate)
//
//  관측 모델: z = H*x + noise
//    H = [0 0 0 1 0 0]   (vx)
//        [0 0 0 0 1 0]   (vy)
//        [0 0 0 0 0 1]   (yaw_rate)
// ════════════════════════════════════════════════════════════════
void EKFPoseEstimator::ekf_update_odom(const OdomMeasurement& odom)
{
    if (!initialized_) return;
    if (!use_odom_vel_) return;

    // 관측 행렬 H (3×6)
    Eigen::Matrix<double, 3, 6> H;
    H.setZero();
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;
    H(2, 5) = 1.0;

    // 측정 노이즈 R (3×3)
    Eigen::Matrix<double, 3, 3> R;
    R.setZero();
    R(0, 0) = r_odom_vel_;
    R(1, 1) = r_odom_vel_;
    R(2, 2) = r_odom_yaw_rate_;

    // 잔차
    Eigen::Vector3d z(odom.vx, odom.vy, odom.yaw_rate);
    Eigen::Vector3d y_res = z - H * x_;

    // 혁신 공분산 S
    Eigen::Matrix<double, 3, 3> S = H * P_ * H.transpose() + R;

    // 칼만 게인 K
    Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    // 상태 업데이트
    x_ = x_ + K * y_res;
    x_(2) = normalize_angle(x_(2));

    // 공분산 업데이트 (Joseph form)
    Matrix6x6 I_KH = Matrix6x6::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

// ════════════════════════════════════════════════════════════════
//  Callbacks
// ════════════════════════════════════════════════════════════════
void EKFPoseEstimator::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    ImuMeasurement imu;
    imu.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    imu.ax        = msg->linear_acceleration.x;
    imu.ay        = msg->linear_acceleration.y;
    imu.yaw_rate  = msg->angular_velocity.z;

    // 초기화 전이면 버퍼만 채움
    if (!initialized_) {
        if (imu_buffer_.size() < IMU_BUF_MAX) imu_buffer_.push_back(imu);
        return;
    }

    // 이미 초기화됐으면 바로 예측
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ekf_predict(imu);
    }
}

void EKFPoseEstimator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    OdomMeasurement odom;
    odom.timestamp  = rclcpp::Time(msg->header.stamp).seconds();
    odom.vx         = msg->twist.twist.linear.x;
    odom.vy         = msg->twist.twist.linear.y;
    odom.yaw_rate   = msg->twist.twist.angular.z;

    if (!initialized_) return;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ekf_update_odom(odom);
    }
}

void EKFPoseEstimator::mcl_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // 쿼터니언 → yaw 변환
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    PoseMeasurement pose;
    pose.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    pose.x         = msg->pose.position.x;
    pose.y         = msg->pose.position.y;
    pose.yaw       = yaw;

    if (!initialized_) {
        // 첫 수신으로 EKF 초기화
        initialize_state(pose);

        // 버퍼에 쌓인 IMU 리플레이 (retroactive predict)
        for (const auto& imu : imu_buffer_) {
            if (imu.timestamp > pose.timestamp) {
                ekf_predict(imu);
            }
        }
        imu_buffer_.clear();
        return;
    }

    // 이후에는 보정(update) 만 수행
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ekf_update_pose(pose);
    }
}

// ════════════════════════════════════════════════════════════════
//  발행 타이머  —  publish_rate Hz로 현재 상태를 발행
// ════════════════════════════════════════════════════════════════
void EKFPoseEstimator::timer_callback()
{
    if (!initialized_) return;
    publish_pose(this->now());
}

void EKFPoseEstimator::publish_pose(const rclcpp::Time& stamp)
{
    double px, py, yaw, vx, vy, yr;
    StateCov P_copy;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        px      = x_(0);
        py      = x_(1);
        yaw     = x_(2);
        vx      = x_(3);
        vy      = x_(4);
        yr      = x_(5);
        P_copy  = P_;
    }

    // ── 쿼터니언 생성 ─────────────────────────────────────
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    // ── /ekf_pose (PoseStamped) ───────────────────────────
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = stamp;
    ps.header.frame_id = map_frame_;
    ps.pose.position.x = px;
    ps.pose.position.y = py;
    ps.pose.position.z = 0.0;
    tf2::convert(q, ps.pose.orientation);
    pose_pub_->publish(ps);

    // ── /ekf_pose_with_covariance ─────────────────────────
    geometry_msgs::msg::PoseWithCovarianceStamped pcs;
    pcs.header = ps.header;
    pcs.pose.pose = ps.pose;
    // 공분산 (6×6 row-major: xx, xy, xθ, xvx, xvy, xω, ...)
    // ROS 표준 pose covariance는 [x,y,z,roll,pitch,yaw] 6×6
    pcs.pose.covariance[0]  = P_copy(0, 0);  // xx
    pcs.pose.covariance[1]  = P_copy(0, 1);  // xy
    pcs.pose.covariance[5]  = P_copy(0, 2);  // x_yaw
    pcs.pose.covariance[6]  = P_copy(1, 0);  // yx
    pcs.pose.covariance[7]  = P_copy(1, 1);  // yy
    pcs.pose.covariance[11] = P_copy(1, 2);  // y_yaw
    pcs.pose.covariance[30] = P_copy(2, 0);  // yaw_x
    pcs.pose.covariance[31] = P_copy(2, 1);  // yaw_y
    pcs.pose.covariance[35] = P_copy(2, 2);  // yaw_yaw
    pose_cov_pub_->publish(pcs);

    // ── /ekf_odom (Odometry) ──────────────────────────────
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id  = base_frame_;
    odom_msg.pose.pose = ps.pose;
    odom_msg.pose.covariance[0]  = P_copy(0, 0);
    odom_msg.pose.covariance[7]  = P_copy(1, 1);
    odom_msg.pose.covariance[35] = P_copy(2, 2);
    odom_msg.twist.twist.linear.x  = vx;
    odom_msg.twist.twist.linear.y  = vy;
    odom_msg.twist.twist.angular.z = yr;
    odom_msg.twist.covariance[0]  = P_copy(3, 3);
    odom_msg.twist.covariance[7]  = P_copy(4, 4);
    odom_msg.twist.covariance[35] = P_copy(5, 5);
    odom_pub_->publish(odom_msg);

    // ── TF broadcast: map → base_link ─────────────────────
    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp    = stamp;
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id  = base_frame_;
        tf_msg.transform.translation.x = px;
        tf_msg.transform.translation.y = py;
        tf_msg.transform.translation.z = 0.0;
        tf2::convert(q, tf_msg.transform.rotation);
        tf_broadcaster_->sendTransform(tf_msg);
    }
}

}  // namespace ekf_pose
