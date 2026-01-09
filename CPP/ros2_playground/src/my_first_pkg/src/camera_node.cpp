#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options)
  : Node("camera_node", options), frame_count_(0)
  {
    // Declare parameters for camera configuration
    this->declare_parameter<int>("camera_device", 0);
    this->declare_parameter<std::string>("camera_vendor", "Logitech");
    this->declare_parameter<std::string>("camera_product", "BRIO");

    std::string vendor = this->get_parameter("camera_vendor").as_string();
    std::string product = this->get_parameter("camera_product").as_string();

    // Try to find camera by vendor/product first
    std::string camera_path = find_camera_by_name(vendor, product);

    if (!camera_path.empty()) {
      RCLCPP_INFO(this->get_logger(), "Found camera: %s", camera_path.c_str());
      camera_.open(camera_path);
    } else {
      // Fall back to device index
      int device = this->get_parameter("camera_device").as_int();
      RCLCPP_WARN(this->get_logger(),
        "No camera matching '%s' + '%s' found, falling back to device %d",
        vendor.c_str(), product.c_str(), device);
      camera_.open(device);
    }

    if (!camera_.isOpened()) {
      RCLCPP_WARN(this->get_logger(), "Failed to open camera - using mock images");
      use_mock_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Camera opened successfully");
      use_mock_ = false;
    }

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(60),
      std::bind(&CameraNode::publish_image, this));

    RCLCPP_INFO(this->get_logger(), "Camera Node component started (publishing every 60 seconds)");
  }

private:
  void publish_image()
  {
    sensor_msgs::msg::Image::SharedPtr msg;

    if (use_mock_) {
      msg = generate_mock_image();
      RCLCPP_INFO(this->get_logger(), "Published mock image frame %u (8x8 rgb8)",
                  frame_count_);
    } else {
      cv::Mat frame;
      if (!camera_.read(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
        return;
      }

      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = "camera_frame";

      msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      RCLCPP_INFO(this->get_logger(), "Published image frame %u (%dx%d bgr8)",
                  frame_count_, frame.cols, frame.rows);
    }

    image_publisher_->publish(*msg);
    frame_count_++;
  }

  sensor_msgs::msg::Image::SharedPtr generate_mock_image()
  {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_frame";
    msg->height = 8;
    msg->width = 8;
    msg->encoding = "rgb8";
    msg->step = msg->width * 3;
    msg->data.resize(msg->height * msg->step);

    // Generate test pattern: colored rows that rotate each frame
    const uint8_t colors[8][3] = {
      {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0},
      {0, 255, 255}, {255, 0, 255}, {255, 255, 255}, {128, 128, 128}
    };

    for (uint32_t row = 0; row < 8; row++) {
      uint32_t color_idx = (row + frame_count_) % 8;
      for (uint32_t col = 0; col < 8; col++) {
        size_t idx = (row * 8 + col) * 3;
        msg->data[idx + 0] = colors[color_idx][0];
        msg->data[idx + 1] = colors[color_idx][1];
        msg->data[idx + 2] = colors[color_idx][2];
      }
    }

    return msg;
  }

  std::string find_camera_by_name(const std::string & vendor, const std::string & product)
  {
    const std::filesystem::path v4l_by_id("/dev/v4l/by-id");

    if (!std::filesystem::exists(v4l_by_id)) {
      RCLCPP_DEBUG(this->get_logger(), "/dev/v4l/by-id does not exist");
      return "";
    }

    for (const auto & entry : std::filesystem::directory_iterator(v4l_by_id)) {
      std::string filename = entry.path().filename().string();

      // Check if filename contains both vendor and product (case-insensitive would be better, but this works)
      if (filename.find(vendor) != std::string::npos &&
          filename.find(product) != std::string::npos &&
          filename.find("index0") != std::string::npos)  // Prefer primary video device
      {
        return entry.path().string();
      }
    }

    // Second pass: accept any matching device if index0 not found
    for (const auto & entry : std::filesystem::directory_iterator(v4l_by_id)) {
      std::string filename = entry.path().filename().string();
      if (filename.find(vendor) != std::string::npos &&
          filename.find(product) != std::string::npos)
      {
        return entry.path().string();
      }
    }

    return "";
  }

  cv::VideoCapture camera_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t frame_count_;
  bool use_mock_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode)
