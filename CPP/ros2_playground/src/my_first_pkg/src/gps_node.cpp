#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <cmath>

using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

class GPSNode : public rclcpp::Node
{
public:
  explicit GPSNode(const rclcpp::NodeOptions & options)
  : Node("gps_node", options), use_mock_(false), mock_tick_(0), serial_fd_(std::nullopt)
  {
    // Declare parameters
    this->declare_parameter<std::string>("serial_port", "");
    this->declare_parameter<int>("baud_rate", 9600);

    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();

    // Try to find and open GPS device
    if (port.empty()) {
      port = find_gps_device();
    }

    if (!port.empty()) {
      serial_fd_ = open_serial_port(port, baud);
      if (serial_fd_ >= 0) {
        RCLCPP_INFO(this->get_logger(), "Opened GPS on %s at %d baud", port.c_str(), baud);
        use_mock_ = false;
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to open %s - using mock GPS data", port.c_str());
        use_mock_ = true;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "No GPS device found - using mock GPS data");
      use_mock_ = true;
    }

    publisher_ = this->create_publisher<NavSatFix>("gps_data", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GPSNode::publish_gps_data, this));

    RCLCPP_INFO(this->get_logger(), "GPS Node component started%s", use_mock_ ? " (mock mode)" : "");
  }

  ~GPSNode()
  {
    if (serial_fd_) {
      close(serial_fd_.value());
    }

    // if (serial_fd_ >= 0) {
    //   close(serial_fd_);
    // }
  }

private:
  rclcpp::Publisher<NavSatFix>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool use_mock_;
  uint32_t mock_tick_;
  std::optional<int> serial_fd_; // Serial port file descriptor
  std::string nmea_buffer_;

  void publish_gps_data()
  {
    auto msg = NavSatFix();
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps_frame";

    if (use_mock_) {
      // Generate mock GPS data - simulate slow movement in San Francisco
      double base_lat = 37.7749;
      double base_lon = -122.4194;
      // Small circular path (~100m radius)
      double radius = 0.001;  // roughly 100m in degrees
      msg.latitude = base_lat + radius * std::sin(mock_tick_ * 0.1);
      msg.longitude = base_lon + radius * std::cos(mock_tick_ * 0.1);
      msg.altitude = 10.0 + std::sin(mock_tick_ * 0.05) * 2.0;  // slight altitude variation
      msg.status.status = NavSatStatus::STATUS_FIX;
      mock_tick_++;

      RCLCPP_INFO(this->get_logger(), "Publishing mock GPS: lat=%.6f, lon=%.6f",
                  msg.latitude, msg.longitude);
    } else {
      // Read from real GPS
      if (!read_gps_data(msg)) {
        RCLCPP_DEBUG(this->get_logger(), "No valid GPS fix this cycle");
        return;  // Skip publishing if no valid data
      }
      RCLCPP_INFO(this->get_logger(), "Publishing GPS: lat=%.6f, lon=%.6f",
                  msg.latitude, msg.longitude);
    }

    publisher_->publish(msg);
  }

  std::string find_gps_device()
  {
    // Check /dev/serial/by-id for GPS devices
    const std::filesystem::path serial_by_id("/dev/serial/by-id");
    if (std::filesystem::exists(serial_by_id)) {
      for (const auto & entry : std::filesystem::directory_iterator(serial_by_id)) {
        std::string name = entry.path().filename().string();
        // Common GPS chipset identifiers
        if (name.find("u-blox") != std::string::npos ||
            name.find("GPS") != std::string::npos ||
            name.find("gps") != std::string::npos ||
            name.find("GNSS") != std::string::npos ||
            name.find("FT232") != std::string::npos)
        {
          RCLCPP_INFO(this->get_logger(), "Found GPS device: %s", entry.path().c_str());
          return entry.path().string();
        }
      }
    }

    // Fallback: check common serial ports
    for (const auto & port : {"/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"}) {
      if (std::filesystem::exists(port)) {
        RCLCPP_INFO(this->get_logger(), "Found serial port: %s", port);
        return port;
      }
    }

    return "";
  }

  std::optional<int> open_serial_port(const std::string & port, int baud_rate)
  {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s", port.c_str(), strerror(errno));
      return std::nullopt;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
      close(fd);
      return -1;
    }

    // Set baud rate
    speed_t speed;
    switch (baud_rate) {
      case 4800: speed = B4800; break;
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default: speed = B9600;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1 mode, no flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
      close(fd);
      return -1;
    }

    return fd;
  }

  bool read_gps_data(NavSatFix & msg)
  {
    if (!serial_fd_) {
      return false;
    }

    char buf[256];
    ssize_t n = read(serial_fd_.value(), buf, sizeof(buf) - 1);
    if (n <= 0) {
      return false;
    }
    buf[n] = '\0';
    nmea_buffer_ += buf;

    // Look for complete NMEA sentences
    size_t pos;
    while ((pos = nmea_buffer_.find("\r\n")) != std::string::npos) {
      std::string sentence = nmea_buffer_.substr(0, pos);
      nmea_buffer_.erase(0, pos + 2);

      if (parse_nmea_gga(sentence, msg)) {
        return true;
      }
    }

    // Prevent buffer from growing indefinitely
    if (nmea_buffer_.size() > 1024) {
      nmea_buffer_.clear();
    }

    return false;
  }

  bool parse_nmea_gga(const std::string & sentence, NavSatFix & msg)
  {
    // Parse GPGGA or GNGGA sentences
    // Format: $GPGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,geoid,M,age,ref*checksum
    if (sentence.find("GGA") == std::string::npos) {
      return false;
    }

    std::vector<std::string> fields;
    std::stringstream ss(sentence);
    std::string field;
    while (std::getline(ss, field, ',')) {
      fields.push_back(field);
    }

    if (fields.size() < 10) {
      return false;
    }

    // Check fix quality (field 6): 0=no fix, 1=GPS fix, 2=DGPS fix
    int quality = 0;
    try {
      quality = std::stoi(fields[6]);
    } catch (...) {
      return false;
    }
    if (quality == 0) {
      msg.status.status = NavSatStatus::STATUS_NO_FIX;
      return false;
    }

    // Parse latitude (field 2-3): DDMM.MMMM,N/S
    if (fields[2].empty() || fields[3].empty()) return false;
    double lat = parse_nmea_coord(fields[2]);
    if (fields[3] == "S") lat = -lat;

    // Parse longitude (field 4-5): DDDMM.MMMM,E/W
    if (fields[4].empty() || fields[5].empty()) return false;
    double lon = parse_nmea_coord(fields[4]);
    if (fields[5] == "W") lon = -lon;

    // Parse altitude (field 9)
    double alt = 0.0;
    if (!fields[9].empty()) {
      try {
        alt = std::stod(fields[9]);
      } catch (...) {}
    }

    msg.latitude = lat;
    msg.longitude = lon;
    msg.altitude = alt;
    msg.status.status = NavSatStatus::STATUS_FIX;
    msg.status.service = NavSatStatus::SERVICE_GPS;

    return true;
  }

  double parse_nmea_coord(const std::string & coord)
  {
    // NMEA format: DDDMM.MMMM (degrees and minutes)
    // Convert to decimal degrees
    try {
      double raw = std::stod(coord);
      int degrees = static_cast<int>(raw / 100);
      double minutes = raw - (degrees * 100);
      return degrees + (minutes / 60.0);
    } catch (...) {
      return 0.0;
    }
  }
};



RCLCPP_COMPONENTS_REGISTER_NODE(GPSNode)
