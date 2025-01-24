#include <hector_multi_robot_tools/prefix_tf_republisher_node.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<hector_multi_robot_tools::PrefixTfRepublisherNode>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
