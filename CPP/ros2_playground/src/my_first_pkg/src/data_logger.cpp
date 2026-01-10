#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/image.hpp"

struct TimestampedGpsFix {
  sensor_msgs::msg::NavSatFix fix;
  rclcpp::Time time;

  TimestampedGpsFix() = delete;

  TimestampedGpsFix(sensor_msgs::msg::NavSatFix f, rclcpp::Time t)
    : fix(std::move(f)), time(t) {}

};

class DataLogger : public rclcpp::Node
{
public:
  explicit DataLogger(const rclcpp::NodeOptions & options)
  : Node("data_logger", options)
  {
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", 10, std::bind(&DataLogger::gps_callback, this, std::placeholders::_1));

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10, std::bind(&DataLogger::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DataLogger component started");
  }

private:
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    auto new_fix = TimestampedGpsFix(*msg, this->now());
    timestamped_gps_fix_ = std::make_optional(new_fix);

    RCLCPP_INFO(this->get_logger(), "Received GPS: lat=%.4f, lon=%.4f, alt=%.2f",
                msg->latitude, msg->longitude, msg->altitude);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received image: %ux%u, encoding=%s, frame_id=%s, data_size=%zu bytes",
      msg->width, msg->height, msg->encoding.c_str(),
      msg->header.frame_id.c_str(), msg->data.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  std::optional<TimestampedGpsFix> timestamped_gps_fix_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(DataLogger)


