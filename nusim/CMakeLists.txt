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
find_package(visualization_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtlebot_msgs)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs)
find_package(sensor_msgs)
get_filename_component(DIR_ABOVE ../../ ABSOLUTE)

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/Teleport.srv"
  DEPENDENCIES builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

add_executable(nusim src/nusim.cpp)
# target_link_libraries(nusim rclcpp std_msgs std_srvs rcl_interfaces geometry_msgs tf2_ros tf2)
ament_target_dependencies(
  nusim 
  rclcpp 
  std_msgs 
  std_srvs 
  rcl_interfaces
  geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  nuturtlebot_msgs
  sensor_msgs
)
target_link_libraries(nusim "${cpp_typesupport_target}" turtlelib::turtlelib)
ament_export_dependencies(rosidl_default_runtime)

install(
  TARGETS nusim
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch 
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()