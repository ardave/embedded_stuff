cmake_minimum_required(VERSION 3.8)
project(my_first_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Build as shared library for component loading
add_library(my_first_pkg_components SHARED
  src/data_logger.cpp
  src/camera_node.cpp)
target_include_directories(my_first_pkg_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(my_first_pkg_components PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(my_first_pkg_components rclcpp rclcpp_components std_msgs sensor_msgs OpenCV cv_bridge)

# Register the components
rclcpp_components_register_nodes(my_first_pkg_components "DataLogger")
rclcpp_components_register_nodes(my_first_pkg_components "CameraNode")

install(TARGETS my_first_pkg_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
