#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

using namespace std::chrono_literals;

class FixedFrameBroadcaster : public rclcpp::Node {
public:
  FixedFrameBroadcaster() : Node("fixed_frame_tf2_broadcaster") {
    tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&FixedFrameBroadcaster::broadcast_timer_callback, this));
  }

private:
  void broadcast_timer_callback() {
    // rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time now(0, 0);
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now;
    t.header.frame_id = "base_link";
    t.child_frame_id = "imu";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_publisher_->sendTransform(t);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
