#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options)
  : Node("camera_node", options), frame_count_(0)
  {
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(60),
      std::bind(&CameraNode::publish_image, this));

    RCLCPP_INFO(this->get_logger(), "Camera Node component started (publishing every 60 seconds)");
  }

private:
  void publish_image()
  {
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera_frame";
    msg.height = 8;
    msg.width = 8;
    msg.encoding = "rgb8";
    msg.step = msg.width * 3;
    msg.data.resize(msg.height * msg.step);

    generate_mock_image(msg.data, frame_count_);

    image_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published mock image frame %u (8x8 rgb8)", frame_count_);
    frame_count_++;
  }

  void generate_mock_image(std::vector<uint8_t> & data, uint32_t frame_num)
  {
    // Generate a simple test pattern: colored rows with frame number encoded
    // Row colors cycle through: red, green, blue, yellow, cyan, magenta, white, gray
    const uint8_t colors[8][3] = {
      {255, 0, 0},     // red
      {0, 255, 0},     // green
      {0, 0, 255},     // blue
      {255, 255, 0},   // yellow
      {0, 255, 255},   // cyan
      {255, 0, 255},   // magenta
      {255, 255, 255}, // white
      {128, 128, 128}  // gray
    };

    for (uint32_t row = 0; row < 8; row++) {
      // Rotate color based on frame number so pattern changes each frame
      uint32_t color_idx = (row + frame_num) % 8;
      for (uint32_t col = 0; col < 8; col++) {
        size_t idx = (row * 8 + col) * 3;
        data[idx + 0] = colors[color_idx][0];
        data[idx + 1] = colors[color_idx][1];
        data[idx + 2] = colors[color_idx][2];
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t frame_count_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode)
