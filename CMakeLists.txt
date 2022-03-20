cmake_minimum_required(VERSION 3.0.2)
project(arm22_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  message_generation
  ros_control_boilerplate
  hardware_interface
  controller_manager
  rover_utils
)

add_service_files(
  FILES
  serial.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm22_hw_main
  include/arm22_control/arm22_hw_interface.h
  src/arm22_hw_interface.cpp
  src/arm22_hw_main.cpp
)

add_executable(arm22_sim_main
  include/arm22_control/arm22_sim_interface.h
  src/arm22_sim_interface.cpp
  src/arm22_sim_main.cpp
)

add_dependencies(arm22_hw_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm22_hw_main
  ${catkin_LIBRARIES}
)
add_dependencies(arm22_sim_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm22_sim_main
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/serial_echo.py
  scripts/joystick_op.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
