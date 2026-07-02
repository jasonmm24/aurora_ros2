cmake_minimum_required(VERSION 3.8)
project(aurora_r2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../aurora_remote_public/include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aurora_bridge_node src/aurora_bridge_node.cpp)

ament_target_dependencies(aurora_bridge_node
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
)

target_link_libraries(aurora_bridge_node
  ${CMAKE_CURRENT_SOURCE_DIR}/../aurora_remote_public/lib/linux_x86_64/libslamtec_aurora_remote_sdk.so
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  aurora_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../aurora_remote_public/lib/linux_x86_64/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

ament_package()