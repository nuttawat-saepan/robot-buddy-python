cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

include_directories(include include/common include/nlohmann)
link_directories(src)

set(
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Go2 executables only
add_executable(go2_stand_example src/go2/go2_stand_example.cpp src/common/motor_crc.cpp)
add_executable(go2_sport_client src/go2/go2_sport_client.cpp src/common/ros2_sport_client.cpp)
add_executable(go2w_control src/go2/go2w_control.cpp src/common/motor_crc.cpp src/common/ros2_sport_client.cpp)
add_executable(go2_robot_state_client src/go2/go2_robot_state_client.cpp src/common/motor_crc.cpp)
add_executable(go2w_messager src/go2/go2w_messager.cpp src/common/motor_crc.cpp src/common/ros2_sport_client.cpp)

# target dependencies
ament_target_dependencies(go2_stand_example ${DEPENDENCY_LIST})
ament_target_dependencies(go2_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(go2w_control ${DEPENDENCY_LIST})
ament_target_dependencies(go2_robot_state_client ${DEPENDENCY_LIST})
ament_target_dependencies(go2w_messager ${DEPENDENCY_LIST})

# install
install(TARGETS
  go2_stand_example
  go2_sport_client
  go2w_control
  go2_robot_state_client
  go2w_messager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
