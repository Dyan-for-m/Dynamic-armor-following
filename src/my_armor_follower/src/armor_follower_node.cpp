#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ArmorFollowClient : public rclcpp::Node
{
public:
  ArmorFollowClient() : Node("armor_follow_client"), last_pose_received_(false)
  {
    navigate_to_pose_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    while (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for the action server to be available...");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/detector/marker", 10,
      std::bind(&ArmorFollowClient::marker_callback, this, std::placeholders::_1));
  }

private:
  void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    if (msg->markers.empty()) return;

    auto marker = msg->markers[0];
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.pose = marker.pose;
    target_pose.header = marker.header;

    if (is_zero_pose(target_pose.pose)) {
      return;
    }

    if (last_pose_received_ && pose_has_changed(target_pose.pose)) {
      send_goal_to_robot(target_pose.pose);
    }

    last_pose_ = target_pose.pose;
    last_pose_received_ = true;
  }

  bool is_zero_pose(const geometry_msgs::msg::Pose & pose)
  {
    return (
      pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0 &&
      pose.orientation.x == 0 && pose.orientation.y == 0 && pose.orientation.z == 0 &&
      pose.orientation.w == 1);
  }

  bool pose_has_changed(const geometry_msgs::msg::Pose & new_pose)
  {
    return new_pose.position.x != last_pose_.position.x ||
           new_pose.position.y != last_pose_.position.y ||
           new_pose.position.z != last_pose_.position.z ||
           new_pose.orientation.x != last_pose_.orientation.x ||
           new_pose.orientation.y != last_pose_.orientation.y ||
           new_pose.orientation.z != last_pose_.orientation.z ||
           new_pose.orientation.w != last_pose_.orientation.w;
  }

  void send_goal_to_robot(const geometry_msgs::msg::Pose & target_pose)
  {
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "camera_color_optical_frame";
    stamped_pose.header.stamp = this->get_clock()->now();
    stamped_pose.pose = target_pose;

    try {
      geometry_msgs::msg::PoseStamped camera_color_frame_pose;
      tf2::doTransform(
        stamped_pose, camera_color_frame_pose,
        tf_buffer_->lookupTransform(
          "camera_color_frame", "camera_color_optical_frame", tf2::TimePointZero,
          tf2::durationFromSec(1.0)));

      // 修改此处：使用 tf2::TimePointZero 并按照正确的 transform 函数签名调用
      geometry_msgs::msg::PoseStamped final_pose = tf_buffer_->transform(
        camera_color_frame_pose, "base_link",
        tf2::TimePointZero,          // tf2 时间点
        "camera_color_frame",        // fixed_frame
        tf2::durationFromSec(1.0));  // 超时时间

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose = final_pose;
      RCLCPP_INFO(this->get_logger(), "Sending goal to NavigateToPose...");

      auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback =
        std::bind(&ArmorFollowClient::result_callback, this, std::placeholders::_1);
      navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);

    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform pose: %s", e.what());
    }
  }

  void result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &
      result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Goal failed!");
    }
  }

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_subscriber_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::Pose last_pose_;
  bool last_pose_received_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmorFollowClient>());
  rclcpp::shutdown();
  return 0;
}