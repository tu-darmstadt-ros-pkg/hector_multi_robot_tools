#include <hector_multi_robot_tools/prefix_tf_republisher_node.hpp>

namespace {

std::string stripLeadingSlash(const std::string& frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1, frame_id.size() - 1);
  }
  return frame_id;
}

}  // namespace

namespace hector_multi_robot_tools {

PrefixTfRepublisherNode::PrefixTfRepublisherNode()
    : PrefixTfRepublisherNode(rclcpp::NodeOptions())
{}

PrefixTfRepublisherNode::PrefixTfRepublisherNode(const rclcpp::NodeOptions& options)
    : Node("prefix_tf_republisher_node", options)
{
  if (get_effective_namespace() == "/") {
    RCLCPP_FATAL_STREAM(get_logger(), "Node is started in global namespace. This will lead to a topic loop.");
    rclcpp::shutdown();
  }

  rcl_interfaces::msg::ParameterDescriptor frame_prefix_descriptor;
  frame_prefix_descriptor.description = "Frame prefix to prepend on frame_id (default: node namespace)";
  frame_prefix_descriptor.read_only = true;
  std::string default_frame_prefix = stripLeadingSlash(get_effective_namespace());
  frame_prefix_ = declare_parameter("frame_prefix", default_frame_prefix, frame_prefix_descriptor);
  RCLCPP_INFO_STREAM(get_logger(), "Republishing tf frame ids with prefix '" << frame_prefix_ << "'");

  rcl_interfaces::msg::ParameterDescriptor global_frames_descriptor;
  global_frames_descriptor.description = "Global frames are republished without attaching the robot-specific prefix.";
  global_frames_descriptor.read_only = true;
  global_frames_ = declare_parameter("global_frames", std::vector<std::string>(), global_frames_descriptor);

  const std::string input_topic = "input_tf_topic";
  std::optional<rclcpp::QoS> qos;
  rclcpp::Rate rate(10);
  while (!qos) {
    rate.sleep();
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for input topic QoS profile.");
    qos = tryDiscoverQoSProfile(input_topic);
  }

  pub_ = create_publisher<tf2_msgs::msg::TFMessage>("output_tf_topic", qos.value());
  sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      input_topic, qos.value(), std::bind(&PrefixTfRepublisherNode::tfMessageCallback, this, std::placeholders::_1));
}

void PrefixTfRepublisherNode::tfMessageCallback(const tf2_msgs::msg::TFMessage& msg) const
{
  tf2_msgs::msg::TFMessage msg_copy = msg;
  prependFramePrefixToMsg(msg_copy, frame_prefix_);
  pub_->publish(msg_copy);
}

void PrefixTfRepublisherNode::prependFramePrefixToMsg(tf2_msgs::msg::TFMessage& tf_message,
                                                      const std::string& prefix) const
{
  for (auto& msg : tf_message.transforms) {
    msg.child_frame_id = preprendFramePrefix(msg.child_frame_id, prefix);
    msg.header.frame_id = preprendFramePrefix(msg.header.frame_id, prefix);
  }
}

std::string PrefixTfRepublisherNode::preprendFramePrefix(const std::string& frame_id, const std::string& prefix) const
{
  auto it = std::find(global_frames_.begin(), global_frames_.end(), frame_id);
  if (it != global_frames_.end()) {
    // Frame is a global frame
    return frame_id;
  }
  // Frame is a local frame, attach prefix
  return prefix + "/" + frame_id;
}

std::optional<rclcpp::QoS> PrefixTfRepublisherNode::tryDiscoverQoSProfile(const std::string& topic) const
{
  // taken from https://github.com/ros-tooling/topic_tools/blob/rolling/topic_tools/src/tool_base_node.cpp#L81
  // Query QoS info for publishers
  std::vector<rclcpp::TopicEndpointInfo> endpoint_info_vec = this->get_publishers_info_by_topic(topic);
  std::size_t num_endpoints = endpoint_info_vec.size();

  // If there are no publishers, return an empty optional
  if (endpoint_info_vec.empty()) {
    return {};
  }

  // Initialize QoS
  rclcpp::QoS qos{10};
  // Default reliability and durability to value of first endpoint
  qos.reliability(endpoint_info_vec[0].qos_profile().reliability());
  qos.durability(endpoint_info_vec[0].qos_profile().durability());
  // Always use automatic liveliness
  qos.liveliness(rclcpp::LivelinessPolicy::Automatic);

  // Reliability and durability policies can cause trouble with endpoint matching
  // Count number of "reliable" publishers and number of "transient local" publishers
  std::size_t reliable_count = 0u;
  std::size_t transient_local_count = 0u;
  // For duration-based policies, note the largest value to ensure matching all publishers
  rclcpp::Duration max_deadline(0, 0u);
  rclcpp::Duration max_lifespan(0, 0u);
  for (const auto& info : endpoint_info_vec) {
    const auto& profile = info.qos_profile();
    if (profile.reliability() == rclcpp::ReliabilityPolicy::Reliable) {
      reliable_count++;
    }
    if (profile.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      transient_local_count++;
    }
    if (profile.deadline() > max_deadline) {
      max_deadline = profile.deadline();
    }
    if (profile.lifespan() > max_lifespan) {
      max_lifespan = profile.lifespan();
    }
  }

  // If not all publishers have a "reliable" policy, then use a "best effort" policy
  // and print a warning
  if (reliable_count > 0u && reliable_count != num_endpoints) {
    qos.best_effort();
    RCLCPP_WARN_STREAM(this->get_logger(), "Some, but not all, publishers on topic '"
                                               << topic
                                               << "' offer 'reliable' reliability. Falling back to 'best effort' "
                                                  "reliability in order to connect to all publishers.");
  }

  // If not all publishers have a "transient local" policy, then use a "volatile" policy
  // and print a warning
  if (transient_local_count > 0u && transient_local_count != num_endpoints) {
    qos.durability_volatile();
    RCLCPP_WARN_STREAM(this->get_logger(), "Some, but not all, publishers on topic '"
                                               << topic
                                               << "' offer 'transient local' durability. Falling back to 'volatile' "
                                                  "durability in order to connect to all publishers.");
  }

  qos.deadline(max_deadline);
  qos.lifespan(max_lifespan);

  return qos;
}
}  // namespace hector_multi_robot_tools
