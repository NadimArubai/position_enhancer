cmake_minimum_required(VERSION 3.8)
project(position_enhancer_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find GTSAM
find_package(GTSAM REQUIRED)
message(STATUS "GTSAM found: ${GTSAM_FOUND}")
message(STATUS "GTSAM include directories: ${GTSAM_INCLUDE_DIRS}")
message(STATUS "GTSAM libraries: ${GTSAM_LIBRARIES}")

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EnhancePosition.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
