#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

class MocapPosePublisher : public rclcpp::Node {
public:
    MocapPosePublisher() : rclcpp::Node("mocap_pose_publisher") {
        // Parameters
        this->declare_parameter<std::string>("input_topic", "/rigid_bodies");
        this->declare_parameter<std::string>("output_frame", "map");
        this->declare_parameter<std::vector<double>>("rotation", {1.0,0.0,0.0,
                                                                0.0,1.0,0.0,
                                                                0.0,0.0,1.0});
        this->declare_parameter<std::vector<double>>("translation", {0.0, 0.0, 0.0});
        this->declare_parameter<bool>("publish_tf", false);
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        std::string input_topic = this->get_parameter("input_topic").as_string();
        output_frame_ = this->get_parameter("output_frame").as_string();

        std::vector<double> rot;
        this->get_parameter("rotation", rot);
        if (rot.size() != 9)
        {
            RCLCPP_WARN(this->get_logger(), "Parameter 'rotation' must have 9 elements; using identity.");
            R_.setIdentity();
        }
        else
        {
            // Row-major: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
            R_ << rot[0], rot[1], rot[2],
            rot[3], rot[4], rot[5],
            rot[6], rot[7], rot[8];
        }
        std::vector<double> trans;
        this->get_parameter("translation", trans);
        if (trans.size() != 3) {
            RCLCPP_WARN(this->get_logger(), "Parameter 'translation' must have 3 elements; using zero vector.");
            t_.setZero();
        }
        else {
            t_ = Eigen::Vector3d(trans[0], trans[1], trans[2]);
        }
        std::cout << t_ << std::endl;

        if (publish_tf_)
        {
            tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
        }

        // Publisher
        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mocap_pose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

        // Subscriber
        sub_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        input_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&MocapPosePublisher::callback, this, std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(), "mocap_pose_publisher started. Subscribing: %s, Publishing: /mocap_pose", input_topic.c_str());
    }
private:
    void callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
        if (msg->rigidbodies.empty()) {
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No rigidbodies in message.");
            return;
        }
        const auto & rb = msg->rigidbodies[0];
        const auto & pose_in = rb.pose;

        // Full SE(3) on position: p' = R * p + t
        Eigen::Vector3d p(pose_in.position.x, pose_in.position.y, pose_in.position.z);
        Eigen::Vector3d p_out = R_ * p + t_;


        // Rotate orientation: q' = q_R * q_in
        Eigen::Quaterniond q_in(pose_in.orientation.w,
                                pose_in.orientation.x,
                                pose_in.orientation.y,
                                pose_in.orientation.z);
        Eigen::Quaterniond q_R(R_); // assumes R_ is a proper rotation matrix
        Eigen::Quaterniond q_out = q_R * q_in;
        q_out.normalize();

        Eigen::Matrix3d rot_mat = q_out.toRotationMatrix();
        double roll = atan2(rot_mat(2,1), rot_mat(2,2));
        double pitch = -asin(rot_mat(2,0));
        double yaw = atan2(rot_mat(1,0), rot_mat(0,0));


        RCLCPP_INFO(this->get_logger(), "Yaw: %f", yaw * 180 / 3.141592);

        geometry_msgs::msg::PoseStamped out;
        out.header.stamp = this->get_clock()->now();
        out.header.frame_id = output_frame_;
        out.pose.position.x = p_out.x();
        out.pose.position.y = p_out.y();
        out.pose.position.z = p_out.z();
        out.pose.orientation.w = q_out.w();
        out.pose.orientation.x = q_out.x();
        out.pose.orientation.y = q_out.y();
        out.pose.orientation.z = q_out.z();

        pub_->publish(out);

        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.frame_id = 'map';
            tf.child_frame_id = 'base_link';
            tf.header.stamp = now();
            tf.transform.translation.x = p_out.x();
            tf.transform.translation.y = p_out.y();
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = out.pose.orientation;
            tf_pub_->sendTransform(tf);
        }
    }

    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    Eigen::Matrix3d R_ {Eigen::Matrix3d::Identity()};
    Eigen::Vector3d t_ {Eigen::Vector3d::Zero()};
    std::string output_frame_ {"mocap"};
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
    bool publish_tf_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapPosePublisher>());
    rclcpp::shutdown();
    return 0;
}