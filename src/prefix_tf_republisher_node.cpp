#include <hector_multi_robot_tools/prefix_tf_republisher_node.hpp>

namespace hector_multi_robot_tools {
PrefixTfRepublisherNode::PrefixTfRepublisherNode() : PrefixTfRepublisherNode(rclcpp::NodeOptions()) {
}

PrefixTfRepublisherNode::PrefixTfRepublisherNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("prefix_tf_republisher_node", options)
{
    if (get_effective_namespace() == "/") {
        RCLCPP_FATAL_STREAM(get_logger(), "Node is started in global namespace. This will lead to a topic loop.");
        rclcpp::shutdown();
    }

    rcl_interfaces::msg::ParameterDescriptor frame_prefix_descriptor;
    frame_prefix_descriptor.description = "Frame prefix to prepend on frame_id (default: node namespace)";
    frame_prefix_descriptor.read_only = true;
    std::string default_frame_prefix = stripLeadingSlash(get_effective_namespace());
    frame_prefix_ = declare_parameter("frame_prefix", default_frame_prefix);
    RCLCPP_INFO_STREAM(get_logger(), "Republishing tf frame ids with prefix '" << frame_prefix_ << "'");

    pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    sub_ = create_subscription<tf2_msgs::msg::TFMessage>("tf", 10,
        std::bind( &PrefixTfRepublisherNode::tfMessageCallback, this, std::placeholders::_1 ));
}

void PrefixTfRepublisherNode::tfMessageCallback(const tf2_msgs::msg::TFMessage &msg) const {
    tf2_msgs::msg::TFMessage msg_copy = msg;
    prependFramePrefix(msg_copy, frame_prefix_);
    pub_->publish(msg_copy);
}

void PrefixTfRepublisherNode::prependFramePrefix(tf2_msgs::msg::TFMessage &tf_message, std::string prefix) {
    for (auto& msg: tf_message.transforms) {
        msg.child_frame_id = prefix + "/" + msg.child_frame_id;
        msg.header.frame_id = prefix + "/" + msg.header.frame_id;
    }
}

std::string PrefixTfRepublisherNode::stripLeadingSlash(const std::string &frame_id) {
    if (!frame_id.empty() && frame_id.front() == '/') {
        return frame_id.substr(1, frame_id.size() - 1);
    } else {
        return frame_id;
    }
}
}
