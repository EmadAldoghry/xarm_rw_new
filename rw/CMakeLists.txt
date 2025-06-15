cmake_minimum_required(VERSION 3.8)
project(rw)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetProcessing.srv"
)

ament_python_install_package(${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY model
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY srdf
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()