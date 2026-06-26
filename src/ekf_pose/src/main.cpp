#include "ekf_pose/ekf_pose_estimator.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ekf_pose::EKFPoseEstimator>());
    rclcpp::shutdown();
    return 0;
}
