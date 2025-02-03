#ifndef PREFIX_TF_REPUBLISHER_NODE_H
#define PREFIX_TF_REPUBLISHER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace hector_multi_robot_tools {

class PrefixTfRepublisherNode : public rclcpp::Node {
public:
    PrefixTfRepublisherNode();
    PrefixTfRepublisherNode(const rclcpp::NodeOptions& options);
private:
    void tfMessageCallback(const tf2_msgs::msg::TFMessage& msg);

    std::optional<rclcpp::QoS> tryDiscoverQoSProfile(const std::string& topic) const;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;

    std::string frame_prefix_;
};

    void prependFramePrefix(tf2_msgs::msg::TFMessage& tf_message, const std::string& prefix);
    std::string stripLeadingSlash(const std::string& frame_id);
}

#endif
