#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
//#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GPSNode : public rclcpp::Node
{
public:
  explicit GPSNode(const rclcpp::NodeOptions & options)
  : Node("gps_node", options)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_data", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GPSNode::publish_gps_data, this));


    RCLCPP_INFO(this->get_logger(), "GPS Node component started");
  }
private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish_gps_data()
  {
    auto msg = sensor_msgs::msg::NavSatFix();
    msg.latitude = 37.749;
    msg.longitude = -122.4194;
    msg.altitude = 10.0;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps_frame";

    RCLCPP_INFO(this->get_logger(), "Publishing GPS: lat=%.4f, lon=%4f", msg.latitude, msg.longitude);
    publisher_->publish(msg);
  }
};



RCLCPP_COMPONENTS_REGISTER_NODE(GPSNode)
