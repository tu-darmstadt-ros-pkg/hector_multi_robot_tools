#ifndef PREFIX_TF_REPUBLISHER_NODE_H
#define PREFIX_TF_REPUBLISHER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace hector_multi_robot_tools {

class PrefixTfRepublisherNode : public rclcpp::Node
{
public:
  PrefixTfRepublisherNode();
  explicit PrefixTfRepublisherNode(const rclcpp::NodeOptions& options);

private:
  void tfMessageCallback(const tf2_msgs::msg::TFMessage& msg) const;

  void prependFramePrefixToMsg(tf2_msgs::msg::TFMessage& tf_message, const std::string& prefix) const;
  std::string preprendFramePrefix(const std::string& frame_id, const std::string& prefix) const;

  std::optional<rclcpp::QoS> tryDiscoverQoSProfile(const std::string& topic) const;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;

  std::string frame_prefix_;
  std::vector<std::string> global_frames_;
};

}  // namespace hector_multi_robot_tools

#endif
