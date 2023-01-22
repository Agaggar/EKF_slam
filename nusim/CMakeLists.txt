cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/Teleport.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(nusim src/nusim.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")
# target_link_libraries(nusim rclcpp std_msgs)
target_link_libraries(nusim "${cpp_typesupport_target}")
ament_target_dependencies(
  nusim 
  rclcpp 
  std_msgs 
  std_srvs 
  rcl_interfaces
  geometry_msgs
  tf2_ros
  tf2
  rosidl_default_generators
)

install(
  TARGETS nusim
  DESTINATION lib/nusim
)

ament_package()
