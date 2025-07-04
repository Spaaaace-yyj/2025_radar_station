## set OpenVINO_LIBRARIES
set(OpenVINO_LIBRARIES openvino::runtime)

add_compile_options(-g)
cmake_minimum_required(VERSION 3.8)
project(radar_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR "/home/spaaaaace/Code/mid70/2025_radar_station/env/opencv/install/lib/cmake/opencv4")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenVINO REQUIRED)

set(OpenVINO_DIR $ENV{INTEL_OPENVINO_DIR})

include_directories(
  ${OpenVINO_DIR}/runtime/include
)

link_directories(
  ${OpenVINO_DIR}/runtime/lib/intel64  # 注意路径是否正确，确认 lib 目录位置
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(radar_station src/radar_station.cpp src/onnx_box.cpp src/robot.cpp)
target_link_libraries(radar_station ${OpenCV_LIBS})
ament_target_dependencies(radar_station rclcpp sensor_msgs cv_bridge image_transport PCL pcl_conversions OpenVINO)

install(TARGETS
  TARGETS radar_station
	DESTINATION lib/${PROJECT_NAME}
	)

ament_package()
