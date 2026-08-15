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
    // 2026-08-04: /ekf/imu/data(중력보상)로 전환 — 축 컨벤션은 /imu/data와
    // 동일함을 bag 재검증함(docs/ekf-pose-analysis-2026-08.md §2).
    declare_parameter("imu_topic",          "/ekf/imu/data");
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
    // §6 실측 RMSE(0.34~0.40 rad/s, mocap 미분 노이즈 포함이라 상한치) 기반.
    declare_parameter("r_gyro_yaw_rate",    0.14);

    // ── 2026-08-03 실차 bag + mocap GT 검증으로 추가된 파라미터 ──────────
    //  docs/ekf-pose-analysis-2026-08.md 참고. 기본값은 "실측으로 가장 좋았던" 조합.
    declare_parameter("imu_axis_yaw180",    true);
    declare_parameter("use_odom_yaw_rate",  false);

    // MCL pose 지연 보상 (retrodiction, §7.3/P3). §8.5.1(record5)은 컨트롤러
    // 조회 시점 기준 MCL pose 나이 26~46ms를 측정했지만, 0730 bag(22:52:38,
    // /ekf_pose 자체 200Hz 발행 vs /newmcl_pose ~51Hz)로 A/B 실측한 결과
    // **개선이 확인 안 됨** — p50 8.5→9.7cm/p90 20.3→25.1cm 악화, yaw MAE만
    // 2.35→2.10° 소폭 개선, 편향은 원래도 거의 0(+0.4cm)이라 고칠 대상이 없었음
    // (docs/ekf-pose-analysis-2026-08.md §11). 원인 추정: 이 구성은 MCL이 IMU
    // 대비 빠르고(51Hz vs 100Hz) 재생 시나리오가 §8.5.1의 "컨트롤러 비동기 조회"
    // 패턴과 다름 — retrodiction이 득이 되는 조건(느린 MCL, 비동기 조회 지연)이
    // 이 테스트엔 없었을 수 있음. **기본값 false** — 검증된 이득이 없는 채로
    // 기본 켜두지 않음. 코드/파라미터는 남겨둠(다른 시나리오 재평가 가능하게).
    declare_parameter("retrodiction_enabled", false);
    declare_parameter("retrodiction_window",  0.5);

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
    r_gyro_yaw_rate_   = get_parameter("r_gyro_yaw_rate").as_double();

    imu_axis_yaw180_   = get_parameter("imu_axis_yaw180").as_bool();
    use_odom_yaw_rate_ = get_parameter("use_odom_yaw_rate").as_bool();
    retrodiction_enabled_ = get_parameter("retrodiction_enabled").as_bool();
    retrodiction_window_  = get_parameter("retrodiction_window").as_double();
    RCLCPP_INFO(get_logger(),
        "[EKF Pose] imu_axis_yaw180=%s (IMU ax,ay 부호반전 보정) | use_odom_yaw_rate=%s | "
        "retrodiction=%s (window=%.2fs)",
        imu_axis_yaw180_ ? "true" : "false", use_odom_yaw_rate_ ? "true" : "false",
        retrodiction_enabled_ ? "true" : "false", retrodiction_window_);

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
//  retrodiction 히스토리를 retrodiction_window_ 만큼만 보관 (오래된 것 제거)
// ════════════════════════════════════════════════════════════════
void EKFPoseEstimator::trim_retrodiction_history(double now_t)
{
    while (!imu_hist_.empty() &&
           now_t - imu_hist_.front().timestamp > retrodiction_window_)
        imu_hist_.pop_front();
    while (!state_hist_.empty() &&
           now_t - state_hist_.front().t > retrodiction_window_)
        state_hist_.pop_front();
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
    last_state_stamp_ = rclcpp::Time(
        static_cast<int64_t>(pose.timestamp * 1e9), RCL_ROS_TIME);
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
//  ⚠ 이건 자전거 모델이 아니다 (예전 주석이 틀렸음). 조향각도, 휠베이스도,
//    타이어 모델도 쓰지 않는 **IMU 구동 점질량 데드레커닝**이다.
//    (MPPI 가 쓰는 파세이카 동역학 자전거 모델과는 별개의 모델이다.)
//
//    px  += (vx*cos(yaw) - vy*sin(yaw)) * dt
//    py  += (vx*sin(yaw) + vy*cos(yaw)) * dt
//    yaw += yaw_rate * dt
//    vx  += (ax + vy*yaw_rate) * dt          ← body-frame Coriolis (교과서 부호)
//    vy  += (ay - vx*yaw_rate) * dt
//    yaw_rate = IMU measurement (직접 대입)
//
//  ⚠⚠ Coriolis 부호는 차체 FLU 입력을 전제로 한다. 2026-08-03의 과거 3호차
//     bag은 ax/ay가 반전돼 imu_axis_yaw180=true가 필요했지만, 현재 3호차는
//     firmware identity + 드라이버 publisher 보정 후 imu_frame_test에서 세 축
//     정상으로 검증돼 기본값 false를 사용한다. 과거 bag/장착 변경 시에는 이
//     보정과 Coriolis 부호를 함께 imu_frame_test로 재검증해야 한다.
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

    // Legacy bag compatibility: current car-3 /imu/data is already body FLU.
    const double ax = imu_axis_yaw180_ ? -imu.ax : imu.ax;
    const double ay = imu_axis_yaw180_ ? -imu.ay : imu.ay;

    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    // ── 비선형 상태 전이 f(x) ─────────────────────────────
    double px_new   = px + (vx * cy - vy * sy) * dt;
    double py_new   = py + (vx * sy + vy * cy) * dt;
    double yaw_new  = normalize_angle(yaw + yr * dt);
    double vx_new   = vx + (ax + vy * yr) * dt;
    double vy_new   = vy + (ay - vx * yr) * dt;
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

    //  ∂vx/∂vy = +yr*dt   (교과서 Coriolis 부호에 맞춤)
    F(3, 4) =  yr * dt;
    //  ∂vy/∂vx = -yr*dt
    F(4, 3) = -yr * dt;

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

    // ── yaw_rate 공분산 정합성 보정 (§7.1) ─────────────────
    // x_(5) 는 위에서 이전 상태와 무관하게 IMU 값으로 직접 대입됐다(직접
    // 대입, 블렌딩 아님). F*P*F'+Q 는 "yr 이 이전 값에서 매끄럽게 이어졌다"고
    // 가정해 전파하므로 위 대입과 모순된다 — 대입 직후 5행/5열을 "방금 들어온
    // 독립된 자이로 측정치"로 리셋한다. F(2,5)=dt 교차항이 여전히 "새 yr 이
    // 이전 x_(5) 에서 유도된다"는 근사를 쓰는 점은 남아있지만(§9 P2, 완전한
    // 해는 아님), 지금보다는 정합적이다.
    P_.row(5).setZero();
    P_.col(5).setZero();
    P_(5, 5) = r_gyro_yaw_rate_;
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
//  EKF Update Step — Odometry (vx, vy[, yaw_rate])
//
//  관측 모델: z = H*x + noise
//    H = [0 0 0 1 0 0]   (vx)
//        [0 0 0 0 1 0]   (vy)        ← /odom 의 vy 는 항상 0 하드코딩.
//                                       현재는 못 쓰는 IMU 적분 vy 를 0 으로
//                                       묶어주는 역할이라 유지하는 게 낫다.
//        [0 0 0 0 0 1]   (yaw_rate)  ← use_odom_yaw_rate=false 면 제외
//
//  /odom 의 angular.z 는 측정이 아니라 vesc_to_odom 의 운동학 재구성
//  (v*tan(delta)/L) 이다. 실차 bag + mocap 검증 결과 **코너에서 실제의
//  0.70~0.80배이고 115ms 늦다.** 반면 IMU 자이로는 0.85~0.95배에 5~15ms 다.
//  이 행을 융합하면 신선하고 정확한 자이로 값이 묵고 낮은 값에 끌려가므로
//  기본값은 제외(use_odom_yaw_rate=false)다. 실측 개선:
//    위치 p50 9.37→8.52cm, yaw p50 3.20°→2.17°, yaw p90 7.80°→5.08°
//  상세: docs/ekf-pose-analysis-2026-08.md §6, §9
// ════════════════════════════════════════════════════════════════
void EKFPoseEstimator::ekf_update_odom(const OdomMeasurement& odom)
{
    if (!initialized_) return;
    if (!use_odom_vel_) return;

    const int m = use_odom_yaw_rate_ ? 3 : 2;   // 관측 차원

    // 관측 행렬 H (m×6)
    Eigen::Matrix<double, Eigen::Dynamic, 6> H(m, 6);
    H.setZero();
    H(0, 3) = 1.0;   // vx
    H(1, 4) = 1.0;   // vy
    if (use_odom_yaw_rate_) H(2, 5) = 1.0;

    // 측정 노이즈 R (m×m)
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
    R(0, 0) = r_odom_vel_;
    R(1, 1) = r_odom_vel_;
    if (use_odom_yaw_rate_) R(2, 2) = r_odom_yaw_rate_;

    // 잔차
    Eigen::VectorXd z(m);
    z(0) = odom.vx;
    z(1) = odom.vy;
    if (use_odom_yaw_rate_) z(2) = odom.yaw_rate;
    Eigen::VectorXd y_res = z - H * x_;

    // 혁신 공분산 S
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;

    // 칼만 게인 K
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

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
        last_state_stamp_ = msg->header.stamp;

        if (retrodiction_enabled_) {
            imu_hist_.push_back(imu);
            state_hist_.push_back({imu.timestamp, x_, P_});
            trim_retrodiction_history(imu.timestamp);
        }
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

        // 버퍼에 쌓인 IMU 리플레이 (retroactive predict).
        // initialize_state() 가 이미 initialized_=true 로 세팅한 뒤라, 이 루프가
        // 락 없이 x_/P_ 를 건드리면 그 사이 들어온 새 imu_callback(락을 잡고
        // ekf_predict 를 부름)과 경합할 수 있다 — 리플레이 구간 전체를 같은
        // 뮤텍스로 감싼다 (docs/ekf-pose-analysis-2026-08.md §7.5/P3).
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (const auto& imu : imu_buffer_) {
                if (imu.timestamp > pose.timestamp) {
                    ekf_predict(imu);
                    last_state_stamp_ = rclcpp::Time(
                        static_cast<int64_t>(imu.timestamp * 1e9), RCL_ROS_TIME);
                }
            }
        }
        imu_buffer_.clear();
        return;
    }

    // 이후에는 보정(update) — MCL pose 지연 보상 (retrodiction, §7.3/P3).
    //
    // MCL pose 는 스캔 취득→파티클필터 연산을 거친 **과거 시점**의 추정이다
    // (§8.5.1 실측: 컨트롤러 조회 시점 기준 26~46ms 나이). 그런데 지금 x_/P_ 는
    // 그 사이 들어온 IMU 로 이미 더 미래로 predict 돼 있다. 이 상태에 pose 를
    // 그냥 update 하면 "과거 시점 측정치를 미래 시점 상태에 잘못 정합"하는
    // 셈이 된다 — retrodiction 은 (1) pose.timestamp 시점의 과거 상태로
    // 되감고 (2) 거기서 보정한 뒤 (3) 그 이후 IMU 를 다시 순서대로 재적용해
    // "현재"로 되돌린다.
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        const StateSnapshot* snap = nullptr;
        if (retrodiction_enabled_) {
            // state_hist_ 는 시간순 정렬 — pose.timestamp 이하인 것 중 가장 최근
            for (const auto& s : state_hist_) {
                if (s.t <= pose.timestamp) snap = &s;
                else break;
            }
        }

        if (snap) {
            x_ = snap->x;
            P_ = snap->P;
            last_predict_time_ = snap->t;

            ekf_update_pose(pose);   // 과거 시점에서 보정

            // 그 이후 IMU 를 순서대로 재적용해 "현재"로 재전진.
            // state_hist_ 는 이 보정으로 궤적이 바뀌었으므로 다시 채운다
            // (imu_hist_ 는 원시 센서 로그라 그대로 재사용 가능).
            std::deque<StateSnapshot> rebuilt;
            for (const auto& imu : imu_hist_) {
                if (imu.timestamp > pose.timestamp) {
                    ekf_predict(imu);
                    rebuilt.push_back({imu.timestamp, x_, P_});
                }
            }
            state_hist_.swap(rebuilt);

            double last_t = rebuilt.empty() ? pose.timestamp : rebuilt.back().t;
            last_state_stamp_ = rclcpp::Time(
                static_cast<int64_t>(last_t * 1e9), RCL_ROS_TIME);
        } else {
            // retrodiction 꺼짐, 또는 pose 가 히스토리 창(retrodiction_window_)
            // 보다 오래돼 되감을 스냅샷이 없음 — 예전처럼 현재 상태에 직접 보정.
            ekf_update_pose(pose);
        }
    }
}

// ════════════════════════════════════════════════════════════════
//  발행 타이머  —  publish_rate Hz로 현재 상태를 발행
// ════════════════════════════════════════════════════════════════
void EKFPoseEstimator::timer_callback()
{
    if (!initialized_) return;
    rclcpp::Time stamp;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stamp = last_state_stamp_;
    }
    publish_pose(stamp);
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
