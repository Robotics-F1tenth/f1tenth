#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class TfTest : public rclcpp::Node
{
public:
  TfTest()
  : Node("tf_test")
  {
    
    start_frame_ = this->declare_parameter<std::string>("start_frame", "base_link0");
    end_frame_ = this->declare_parameter<std::string>("end_frame", "laser_model0");
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      1s, std::bind(&TfTest::on_timer, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string start_frame_;
  std::string end_frame_;
  void on_timer()
  {

    geometry_msgs::msg::TransformStamped t;
    try {
    t = tf_buffer_->lookupTransform(
        start_frame_.c_str(), end_frame_.c_str(),
        tf2::TimePointZero);
    RCLCPP_INFO(
        this->get_logger(), "Transform from %s to %s: "
        "Translation: [%f, %f, %f], Rotation: [%f, %f, %f, %f]",
        start_frame_.c_str(), end_frame_.c_str(),
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        start_frame_.c_str(), end_frame_.c_str(), ex.what());
    return;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfTest>());
  rclcpp::shutdown();
  return 0;
}