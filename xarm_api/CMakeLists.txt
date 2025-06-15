cmake_minimum_required(VERSION 3.5)
project(xarm_api)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter 
    -Wno-unused-variable
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_sdk REQUIRED)

include_directories(
  include
  ${xarm_sdk_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)

set(dependencies "rclcpp" "rclcpp_action" "std_msgs" "sensor_msgs" "control_msgs" "geometry_msgs" "xarm_msgs")

add_library(xarm_ros_driver SHARED
  src/xarm_driver.cpp
)
ament_target_dependencies(xarm_ros_driver ${dependencies})
target_link_libraries(xarm_ros_driver 
  ${xarm_sdk_LIBRARIES}
  ${ament_LIBRARIES}
)

add_library(xarm_ros_client SHARED
  src/xarm_ros_client.cpp
)
ament_target_dependencies(xarm_ros_client ${dependencies})

ament_export_libraries(xarm_ros_driver)
ament_export_libraries(xarm_ros_client)
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS xarm_ros_driver xarm_ros_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()