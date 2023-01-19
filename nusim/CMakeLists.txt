cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(nusim src/nusim.cpp)
# target_link_libraries(nusim rclcpp std_msgs)
ament_target_dependencies(nusim rclcpp std_msgs)

install(
  TARGETS nusim
  DESTINATION lib/nusim
)

ament_package()
