cmake_minimum_required(VERSION 3.8)
project(turtle_control)

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
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
get_filename_component(DIR_ABOVE ../../ ABSOLUTE)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "${DIR_ABOVE}/nuturtlebot_msgs:msg/SensorData.msg"
  "${DIR_ABOVE}/nuturtlebot_msgs:msg/WheelCommands.msg"
  DEPENDENCIES builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

add_executable(turtle_control src/turtle_control.cpp)
# target_include_directories(turtle_control PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(turtle_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  turtle_control
  rclcpp 
  std_msgs 
  std_srvs 
  rcl_interfaces
  geometry_msgs
  builtin_interfaces
  sensor_msgs
)

target_link_libraries(turtle_control "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)

install(
  TARGETS turtle_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
