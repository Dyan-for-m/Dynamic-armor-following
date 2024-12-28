#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class ArmorFollowerClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ArmorFollowerClient()
  : Node("armor_follower_client"),
    goal_active_(false),
    first_pose_(true),
    latest_pose_available_(false)
  {
    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/detector/marker", 10,
      std::bind(&ArmorFollowerClient::markerCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ =
      this->create_wall_timer(1s, std::bind(&ArmorFollowerClient::sendGoalIfAvailable, this));
  }

private:
  void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    if (msg->markers.empty()) {
      return;
    }

    // 取第一个marker作为装甲板位置
    auto marker = msg->markers[0];
    // marker.pose在frame_id = "camera_color_optical_frame"下

    geometry_msgs::msg::PoseStamped pose_in_optical;
    pose_in_optical.header = marker.header;
    pose_in_optical.pose = marker.pose;

    // 使用lookupTransform查找camera_color_frame和camera_color_optical_frame之间的变换
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        "camera_color_frame", "camera_color_optical_frame", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF lookupTransform failed: %s", ex.what());
      return;
    }

    // 使用transform将pose从camera_color_optical_frame变换到camera_color_frame
    geometry_msgs::msg::PoseStamped pose_in_camera_frame;
    try {
      pose_in_camera_frame =
        tf_buffer_->transform(pose_in_optical, "camera_color_frame", tf2::durationFromSec(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
      return;
    }

    // 使用transform将pose从camera_color_frame变换到base_link
    geometry_msgs::msg::PoseStamped pose_in_base;
    try {
      pose_in_base =
        tf_buffer_->transform(pose_in_camera_frame, "base_link", tf2::durationFromSec(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
      return;
    }

    // 检查detector/marker是否有数据
    if (
      pose_in_base.pose.position.x == 0.0 && pose_in_base.pose.position.y == 0.0 &&
      pose_in_base.pose.position.z == 0.0 && pose_in_base.pose.orientation.x == 0.0 &&
      pose_in_base.pose.orientation.y == 0.0 && pose_in_base.pose.orientation.z == 0.0 &&
      pose_in_base.pose.orientation.w == 1.0) {
      RCLCPP_WARN(get_logger(), "No valid data from detector/marker");
      return;
    }

    // 简易低通滤波提高稳定性
    if (first_pose_) {
      filtered_pose_ = pose_in_base;
      first_pose_ = false;
    } else {
      double alpha = 0.3;  // 滤波系数，可根据实际调参
      filtered_pose_.pose.position.x =
        alpha * pose_in_base.pose.position.x + (1 - alpha) * filtered_pose_.pose.position.x;
      filtered_pose_.pose.position.y =
        alpha * pose_in_base.pose.position.y + (1 - alpha) * filtered_pose_.pose.position.y;
      filtered_pose_.pose.position.z =
        alpha * pose_in_base.pose.position.z + (1 - alpha) * filtered_pose_.pose.position.z;

      // 姿态如果需要滤波也同理，这里简化不对姿态滤波
      filtered_pose_.pose.orientation = pose_in_base.pose.orientation;
    }

    filtered_pose_.header.stamp = this->now();
    filtered_pose_.header.frame_id = "base_link";
    latest_pose_available_ = true;
  }

  void sendGoalIfAvailable()
  {
    if (!latest_pose_available_) {
      return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Nav2 action server not available");
      return;
    }

    // 给机器人一个稍微前方的位置，以实现跟随
    geometry_msgs::msg::PoseStamped goal_pose = filtered_pose_;
    goal_pose.pose.position.x += 0.1;  // 前方0.1米，可根据需求调节

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    // 取消之前正在进行的goal（如果有），以更新新的目标点
    if (goal_active_) {
      nav_client_->async_cancel_all_goals();
    }

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const GoalHandleNavigateToPose::WrappedResult & result) {
        goal_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal finished with status: %d", result.code);
      };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
    goal_active_ = true;
    latest_pose_available_ = false;  // 等待下次更新
  }

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool goal_active_;
  bool first_pose_;
  bool latest_pose_available_;
  geometry_msgs::msg::PoseStamped filtered_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmorFollowerClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
