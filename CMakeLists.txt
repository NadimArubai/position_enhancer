cmake_minimum_required(VERSION 3.8)
project(position_enhancer)

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
find_package(gtsam_msgs REQUIRED)
#find_package(Eigen3 CONFIG REQUIRED) # need to find again as REQUIRED
#get_target_property(GTSAM_EIGEN_INCLUDE_FOR_BUILD Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

# Find GTSAM
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
message(STATUS "GTSAM found: ${GTSAM_FOUND}")
message(STATUS "GTSAM include directories: ${GTSAM_INCLUDE_DIRS}")
message(STATUS "GTSAM libraries: ${GTSAM_LIBRARIES}")

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EnhancePosition.srv"
  DEPENDENCIES geometry_msgs
)
#ament_package()


#find_package(gtsam_msgs REQUIRED)                         # CHANGE

# Stateless solver node
add_executable(position_solver_node
  src/position_solver_node.cpp
)

ament_target_dependencies(position_solver_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  gtsam_msgs
#  ${PROJECT_NAME}
#  GTSAM
#  Eigen3
)

target_link_libraries(position_solver_node
#    ${Eigen3_LIBRARIES} 
#    ${GTSAM_LIBRARIES}
    gtsam
)

install(TARGETS
  position_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Client node
#add_executable(position_solver_client_node
#  src/position_solver_client_node.cpp
#)

#ament_target_dependencies(position_solver_client_node
#  rclcpp
#  geometry_msgs
#  std_msgs
#  tf2
#  tf2_geometry_msgs
#)

#target_link_libraries(position_solver_client_node
#  GTSAM::gtsam
#)

#install(TARGETS
#  position_solver_client_node
#  DESTINATION lib/${PROJECT_NAME}
#)


#install(DIRECTORY
#  srv
#  DESTINATION share/${PROJECT_NAME}/
#)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
