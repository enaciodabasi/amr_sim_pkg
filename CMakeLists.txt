cmake_minimum_required(VERSION 3.5)
project(amr_sim_pkg)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs)
find_package(sensor_msgs)

include_directories(include)

install(
  DIRECTORY config include launch maps meshes models params rviz src urdf worlds
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
