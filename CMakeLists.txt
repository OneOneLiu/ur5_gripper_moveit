cmake_minimum_required(VERSION 3.22)
project(ur5_gripper_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    rclcpp
    rclcpp_action
    rviz_visual_tools
    Boost
    Eigen3
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Keyboard control example for servo
add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies(servo_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Planning Scene Tutorial
add_executable(ignore_certain_collisions src/ignore_certain_collisions.cpp)
target_include_directories(ignore_certain_collisions PUBLIC include)
ament_target_dependencies(ignore_certain_collisions ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# Install Binaries
install(
  TARGETS servo_keyboard_input
  ignore_certain_collisions
  DESTINATION lib/${PROJECT_NAME}
)