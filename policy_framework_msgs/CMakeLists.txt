cmake_minimum_required(VERSION 3.16)
project(policy_framework_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ActivePolicies.msg
  msg/MockFault.msg
  msg/PolicyInfo.msg
  msg/SafetyStatus.msg
  msg/SafetyStatusArray.msg
  srv/ListPolicies.srv
  srv/SwitchPolicy.srv
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
