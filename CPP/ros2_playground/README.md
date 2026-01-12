# ROS2 Sensor Fusion Playground

A ROS2 workspace exploring camera integration and sensor data logging, designed for eventual deployment on robotic platforms.

## Nodes

### CameraNode
Captures images from a USB camera and publishes them as ROS2 Image messages.

**Features:**
- Automatic camera detection by vendor/product name (via `/dev/v4l/by-id`)
- Fallback to device index if named camera not found
- Mock image generation when no camera available (for testing)
- OpenCV integration via cv_bridge
- Configurable capture interval (default: 60 seconds)

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_device` | 0 | Fallback device index |
| `camera_vendor` | "Logitech" | Camera vendor to search for |
| `camera_product` | "BRIO" | Camera product name |

**Published Topics:**
- `camera/image_raw` (sensor_msgs/Image)

### DataLogger
Subscribes to GPS and camera data for eventual logging/fusion.

**Subscribed Topics:**
- `/fix` (sensor_msgs/NavSatFix) - GPS data from nmea_navsat_driver
- `camera/image_raw` (sensor_msgs/Image) - Camera frames

**Features:**
- Timestamps GPS fixes for temporal correlation with images
- Logs received data for debugging

## Building

```bash
# In the ros2_playground directory
source /opt/ros/jazzy/setup.bash  # or your ROS2 distro
colcon build
source install/setup.bash
```

## Running

```bash
# Launch all nodes (requires a launch file) or run individually:
ros2 run my_first_pkg camera_node
ros2 run my_first_pkg data_logger

# For GPS data, also run:
ros2 run nmea_navsat_driver nmea_serial_driver
```

## Development Environment

This project uses a Dev Container for consistent development:
- ROS2 Jazzy base image
- OpenCV and cv_bridge pre-installed
- VS Code integration

## Future Plans

- Image + GPS data logging to disk
- Geotagged image export
- Integration with navigation stack
