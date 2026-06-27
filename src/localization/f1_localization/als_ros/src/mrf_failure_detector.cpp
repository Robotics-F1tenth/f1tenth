#include "rclcpp/rclcpp.hpp"
#include <als_ros/MRFFailureDetector.h>

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "mrf_failure_detector");

//     als_ros::MRFFD detector;
//     double failureDetectionHz = detector.getFailureDetectionHz();
//     ros::Rate loopRate(failureDetectionHz);
//     while (ros::ok()) {
//         ros::spinOnce();
//         detector.setCanUpdateResidualErrors(false);
//         detector.predictFailureProbability();
//         detector.publishROSMessages();
//         detector.setCanUpdateResidualErrors(true);
//         detector.printFailureProbability();
//         loopRate.sleep();
//     }

//     return 0;
// }
class MRFFailureDetectorNode : public rclcpp::Node {
    public:
        MRFFailureDetectorNode() : Node("mrf_failure_detector") {
            mrf_fd_ = std::make_shared<als_ros::MRFFD>();
            failure_detection_hz_ = mrf_fd_->getFailureDetectionHz();
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / failure_detection_hz_),
                std::bind(&MRFFailureDetectorNode::timerCallback, this)
            );
        }
    
    private:
        void timerCallback() {
            mrf_fd_->setCanUpdateResidualErrors(false);
            mrf_fd_->predictFailureProbability();
            mrf_fd_->publishROSMessages();
            mrf_fd_->setCanUpdateResidualErrors(true);
            mrf_fd_->printFailureProbability();
        }

        std::shared_ptr<als_ros::MRFFD> mrf_fd_;
        double failure_detection_hz_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MRFFailureDetectorNode>());
    rclcpp::shutdown();
    return 0;
}