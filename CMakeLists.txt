cmake_minimum_required(VERSION 3.5)

project(high_freq_pub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(topics_library_high_freq SHARED
  src/talker.cpp
  src/listener.cpp)

target_compile_definitions(topics_library_high_freq
  PRIVATE "HIGH_FREQ_PUB_BUILDING_DLL")
ament_target_dependencies(topics_library_high_freq
  "rclcpp"
  "rclcpp_components"
  "std_msgs")

rclcpp_components_register_node(topics_library_high_freq
  PLUGIN "high_freq_pub::Talker"
  EXECUTABLE talker)

rclcpp_components_register_node(topics_library_high_freq
  PLUGIN "high_freq_pub::Listener"
  EXECUTABLE listener)

install(TARGETS
  topics_library_high_freq
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
