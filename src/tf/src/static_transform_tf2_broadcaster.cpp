#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class StaticFramePublisher : public rclcpp::Node
{
public:
  StaticFramePublisher() : Node("static_transform_publisher")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 设置变换参数
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "camera_link";

    transformStamped.transform.translation.x = 0.04;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.18;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);  // 没有旋转
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // 发布静态变换
    tf_static_broadcaster_->sendTransform(transformStamped);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticFramePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}