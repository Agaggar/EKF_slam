cmake_minimum_required(VERSION 3.8)
project(nuslam)

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
find_package(nuturtle_control REQUIRED)
find_package(Armadillo)
find_package(visualization_msgs REQUIRED)

add_executable(slam src/ekf_slam.cpp)
# target_include_directories(ekf_slam PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(ekf_slam PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  slam
  rclcpp 
  std_msgs 
  std_srvs 
  rcl_interfaces
  geometry_msgs
  builtin_interfaces
  sensor_msgs
  nuturtlebot_msgs
  nusim
  nuturtle_control
  visualization_msgs
  tf2_ros
  tf2
)

target_link_libraries(slam turtlelib::turtlelib)

install(TARGETS slam
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
