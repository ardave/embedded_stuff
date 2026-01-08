#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options)
  : Node("camera_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Camera Node component started");
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode)
