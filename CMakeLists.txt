cmake_minimum_required(VERSION 3.5)
project(youbot_driver_ros_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(youbot_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(brics_actuator REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system)


## preprocessor definitions
add_definitions(-DYOUBOT_CONFIGURATIONS_DIR=${youbot_driver_CONFIG_DIR})

## Set Compiler flags
include_directories(
	include
	${Boost_INCLUDE_DIR}
)


add_executable(youbot_driver_ros_interface
    src/youbot_oodl.cpp
    src/YouBotOODLWrapper.cpp
    src/YouBotConfiguration.cpp
    src/joint_state_observer_oodl.cpp
)

ament_target_dependencies(youbot_driver_ros_interface
  youbot_driver
  rclcpp
  brics_actuator
  tf2
  tf2_ros
  control_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)

target_link_libraries(youbot_driver_ros_interface ${Boost_LIBRARIES})

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)


## run sudo command to enable direct network access
## disabled here by default because it is set by Debian postinst command
option(${PROJECT_NAME}_USE_SETCAP "Set permissions to access ethernet interface without sudo" OFF)

set(SUDO_COMMAND sudo)

if(${PROJECT_NAME}_USE_SETCAP)
    add_custom_command(TARGET youbot_driver_ros_interface POST_BUILD
    COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep $<TARGET_FILE:youbot_driver_ros_interface>
)
endif()


#test programs
add_executable(youbot_arm_test
    src/examples/youbot_arm_test.cpp
)
ament_target_dependencies(youbot_arm_test
  youbot_driver
  rclcpp
  brics_actuator
  tf2
  tf2_ros
  control_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)
target_link_libraries(youbot_arm_test ${Boost_LIBRARIES} ${youbot_driver_LIBRARIES})

add_executable(youbot_2nd_arm_test
	src/examples/youbot_2nd_arm_test.cpp
)
ament_target_dependencies(youbot_2nd_arm_test
  youbot_driver
  rclcpp
  brics_actuator
  tf2
  tf2_ros
  control_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)
target_link_libraries(youbot_2nd_arm_test ${Boost_LIBRARIES} ${youbot_driver_LIBRARIES})

#install(TARGETS youbot_driver_ros_interface youbot_arm_test
install(TARGETS youbot_2nd_arm_test youbot_arm_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# enable python teleop script
ament_python_install_package(src/examples/)

install(PROGRAMS
    src/examples/youbot_keyboard_teleop.py
    debian/postinst
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  youbot_driver
  rclcpp
  brics_actuator
  tf2
  tf2_ros
  control_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)

ament_package()
