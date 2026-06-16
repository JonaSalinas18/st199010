cmake_minimum_required(VERSION 3.8)
project(st199010)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# =========================
# Core library
# =========================
add_library(lidar_odometry_core
  src/KdTree.cpp
  src/PointCloudProcessor.cpp
)

target_include_directories(lidar_odometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lidar_odometry_core
  Eigen3
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

# =========================
# ROS2 executable node
# =========================
add_executable(point_cloud_processor_node
  src/main.cpp
)

target_link_libraries(point_cloud_processor_node
  lidar_odometry_core
)

ament_target_dependencies(point_cloud_processor_node
  rclcpp
)

# =========================
# Install
# =========================
install(TARGETS
  lidar_odometry_core
  point_cloud_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(KdTreeTest
    test/KdTreeTest.cpp
  )

  target_link_libraries(KdTreeTest
    lidar_odometry_core
  )

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
