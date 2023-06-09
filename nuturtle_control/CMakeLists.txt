cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

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
find_package(turtlelib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(nusim REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "srv/Circle.srv"
  "srv/Reverse.srv"
  "srv/Stop.srv"
  DEPENDENCIES builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

add_executable(turtle_control src/turtle_control.cpp)
# target_include_directories(nuturtle_control PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(nuturtle_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  turtle_control
  rclcpp 
  std_msgs 
  std_srvs 
  rcl_interfaces
  geometry_msgs
  builtin_interfaces
  sensor_msgs
  nuturtlebot_msgs
  nusim
)

target_link_libraries(turtle_control "${cpp_typesupport_target}" turtlelib::turtlelib)
ament_export_dependencies(rosidl_default_runtime)

add_executable(odometry src/odometry.cpp)

ament_target_dependencies(
  odometry
  rclcpp 
  std_msgs 
  std_srvs 
  rcl_interfaces
  tf2_ros
  tf2
  sensor_msgs
  tf2_geometry_msgs
  nav_msgs
  nusim
)
target_link_libraries(odometry "${cpp_typesupport_target}" turtlelib::turtlelib)

add_executable(circle src/circle.cpp)

ament_target_dependencies(
  circle
  rclcpp
  std_msgs
  std_srvs
  rcl_interfaces
  geometry_msgs
  nuturtlebot_msgs
)
target_link_libraries(circle "${cpp_typesupport_target}")

install(
  TARGETS turtle_control odometry circle
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()