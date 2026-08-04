cmake_minimum_required(VERSION 3.8)
project(so_arm_100_stick_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StickSpec.msg"
  "msg/TaskState.msg"
  "action/PickStick.action"
  "action/PlaceStick.action"
  "action/ReleaseStick.action"
  "action/Reset.action"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
