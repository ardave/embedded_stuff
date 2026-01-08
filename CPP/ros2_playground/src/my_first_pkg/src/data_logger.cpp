#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class DataLogger : public rclcpp::Node
{
public:
  explicit DataLogger(const rclcpp::NodeOptions & options)
  : Node("data_logger", options)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps_data", 10, std::bind(&DataLogger::gps_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DataLogger component started");
  }
private:
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received GPS: lat=%.4f, lon=%.4f, alt=%.2f",
                msg->latitude, msg->longitude, msg->altitude);
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(DataLogger)
