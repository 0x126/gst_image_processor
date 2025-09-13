cmake_minimum_required(VERSION 3.8)
project(gst_image_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0>=1.14)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0>=1.14)

# Include directories
include_directories(
  include
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${GSTREAMER_VIDEO_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GSTREAMER_APP_LIBRARY_DIRS}
  ${GSTREAMER_VIDEO_LIBRARY_DIRS}
)

# Add library
add_library(gst_v4l2_processor SHARED
  src/v4l2_processor.cpp
  src/v4l2_processor_impl.cpp
)

target_compile_features(gst_v4l2_processor PUBLIC c_std_99 cxx_std_17)
target_compile_options(gst_v4l2_processor PRIVATE ${GSTREAMER_CFLAGS_OTHER})

target_link_libraries(gst_v4l2_processor
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${GSTREAMER_VIDEO_LIBRARIES}
  ${OpenCV_LIBS}
)

# Create ROS2 component
add_library(gst_image_processor_component SHARED
  src/gst_image_processor_node.cpp
)

target_compile_features(gst_image_processor_component PUBLIC c_std_99 cxx_std_17)

target_link_libraries(gst_image_processor_component
  gst_v4l2_processor
)

ament_target_dependencies(gst_image_processor_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  image_transport
  camera_info_manager
  cv_bridge
  OpenCV
)

rclcpp_components_register_node(
  gst_image_processor_component
  PLUGIN "gst_image_processor::GstImageProcessorNode"
  EXECUTABLE gst_image_processor_node
)

# Install targets
install(TARGETS
  gst_v4l2_processor
  gst_image_processor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(gst_v4l2_processor gst_image_processor_component)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  camera_info_manager
  cv_bridge
  OpenCV
)

ament_package()