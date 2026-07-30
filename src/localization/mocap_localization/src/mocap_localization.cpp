#include <memory>
#include <string>
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
        this->declare_parameter<std::string>("output_topic", "/mocap_pose");
        this->declare_parameter<std::string>("rigid_body_name", "");
        this->declare_parameter<std::string>("output_frame", "map");
        this->declare_parameter<std::vector<double>>("rotation", {1.0,0.0,0.0,
                                                                0.0,1.0,0.0,
                                                                0.0,0.0,1.0});
        this->declare_parameter<std::vector<double>>("translation", {0.0, 0.0, 0.0});
        // Second stage of the chain calibration_v2.py solves:
        //   T_map_base = T_map_mocap('rotation','translation')
        //                  o T_mocap_point(measured)
        //                  o T_point_base('tracked_point_to_base_*')
        // Identity defaults keep the single-stage behaviour for setups that never ran the
        // joint calibration.
        this->declare_parameter<std::vector<double>>("tracked_point_to_base_rotation", {1.0,0.0,0.0,
                                                                                       0.0,1.0,0.0,
                                                                                       0.0,0.0,1.0});
        this->declare_parameter<std::vector<double>>("tracked_point_to_base_translation", {0.0, 0.0, 0.0});
        this->declare_parameter<bool>("publish_tf", false);
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        // Vertical axis of the incoming mocap stream. "z_up" matches ROS (REP-103) and is
        // applied as identity. "y_up" is the OptiTrack/Motive default; without conversion
        // the body's forward axis lands on ROS -Z and the pose renders stabbed into the
        // ground. Applied BEFORE 'rotation'/'translation', which calibration_v2.py owns
        // and solves as SE(2) (yaw + xy) -- that solver cannot absorb a 90 deg roll.
        this->declare_parameter<std::string>("input_axes", "z_up");

        std::string input_topic = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        rigid_body_name_ = this->get_parameter("rigid_body_name").as_string();
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

        std::vector<double> rot_pb;
        this->get_parameter("tracked_point_to_base_rotation", rot_pb);
        if (rot_pb.size() != 9) {
            RCLCPP_WARN(this->get_logger(),
                "Parameter 'tracked_point_to_base_rotation' must have 9 elements; using identity.");
            R_pb_.setIdentity();
        }
        else {
            R_pb_ << rot_pb[0], rot_pb[1], rot_pb[2],
                     rot_pb[3], rot_pb[4], rot_pb[5],
                     rot_pb[6], rot_pb[7], rot_pb[8];
        }
        q_pb_ = Eigen::Quaterniond(R_pb_);
        q_pb_.normalize();

        std::vector<double> trans_pb;
        this->get_parameter("tracked_point_to_base_translation", trans_pb);
        if (trans_pb.size() != 3) {
            RCLCPP_WARN(this->get_logger(),
                "Parameter 'tracked_point_to_base_translation' must have 3 elements; using zero vector.");
            t_pb_.setZero();
        }
        else {
            t_pb_ = Eigen::Vector3d(trans_pb[0], trans_pb[1], trans_pb[2]);
        }

        const std::string input_axes = this->get_parameter("input_axes").as_string();
        if (input_axes == "y_up") {
            // R_x(+90 deg): (x, y, z)_mocap -> (x, -z, y)_ros, so mocap +Y becomes ROS +Z.
            R_axis_ << 1.0, 0.0,  0.0,
                       0.0, 0.0, -1.0,
                       0.0, 1.0,  0.0;
        } else if (input_axes != "z_up") {
            RCLCPP_WARN(this->get_logger(),
                "Unknown input_axes '%s'; expected \"z_up\" or \"y_up\". Using z_up.",
                input_axes.c_str());
        }
        q_axis_ = Eigen::Quaterniond(R_axis_);
        RCLCPP_INFO(this->get_logger(), "input_axes: %s", input_axes.c_str());

        if (publish_tf_)
        {
            tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
        }

        // Publisher
        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

        // Subscriber
        sub_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        input_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&MocapPosePublisher::callback, this, std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(),
            "mocap_pose_publisher started. Subscribing: %s, body: '%s', publishing: %s",
            input_topic.c_str(), rigid_body_name_.c_str(), output_topic_.c_str());
    }
private:
    void callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
        if (msg->rigidbodies.empty()) {
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No rigidbodies in message.");
            return;
        }
        std::size_t selected_index = 0;
        if (!rigid_body_name_.empty()) {
            bool found = false;
            for (std::size_t i = 0; i < msg->rigidbodies.size(); ++i) {
                if (msg->rigidbodies[i].rigid_body_name == rigid_body_name_) {
                    selected_index = i;
                    found = true;
                    break;
                }
            }
            if (!found) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Rigid body '%s' not found in the current message.",
                    rigid_body_name_.c_str());
                return;
            }
        }

        const auto & rb = msg->rigidbodies[selected_index];
        const auto & pose_in = rb.pose;

        // T_map_mocap o T_mocap_point o T_point_base -- the same chain calibration_v2.py
        // fits, so its two solved stages are both honoured here.
        Eigen::Vector3d p(pose_in.position.x, pose_in.position.y, pose_in.position.z);
        Eigen::Quaterniond q_in(pose_in.orientation.w,
                                pose_in.orientation.x,
                                pose_in.orientation.y,
                                pose_in.orientation.z);
        Eigen::Quaterniond q_R(R_); // assumes R_ is a proper rotation matrix

        // Stage 1: tracked point in the map frame.
        Eigen::Vector3d p1 = R_ * (R_axis_ * p) + t_;
        Eigen::Quaterniond q1 = q_R * (q_axis_ * q_in);
        q1.normalize();

        // Stage 2: tracked point -> base_link. Composed on the right so the offset is
        // rotated into the map frame by the stage-1 orientation.
        Eigen::Vector3d p_out = p1 + q1 * t_pb_;
        Eigen::Quaterniond q_out = q1 * q_pb_;
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
            tf.header.frame_id = "map";
            tf.child_frame_id = "base_link";
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
    std::string output_topic_ {"/mocap_pose"};
    std::string rigid_body_name_;
    Eigen::Matrix3d R_ {Eigen::Matrix3d::Identity()};
    Eigen::Vector3d t_ {Eigen::Vector3d::Zero()};
    Eigen::Matrix3d R_axis_ {Eigen::Matrix3d::Identity()};
    Eigen::Quaterniond q_axis_ {Eigen::Quaterniond::Identity()};
    Eigen::Matrix3d R_pb_ {Eigen::Matrix3d::Identity()};
    Eigen::Quaterniond q_pb_ {Eigen::Quaterniond::Identity()};
    Eigen::Vector3d t_pb_ {Eigen::Vector3d::Zero()};
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
