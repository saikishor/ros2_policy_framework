cmake_minimum_required(VERSION 3.16)
project(policy_framework)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPENDS
  control_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  lifecycle_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(onnxruntime_vendor REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  srv/SwitchPolicy.srv
  LIBRARY_NAME ${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces rosidl_typesupport_cpp)

add_library(${PROJECT_NAME} SHARED
  src/policy_pipeline.cpp
  src/onnx_inference.cpp
  src/topic_sources.cpp
  src/multi_dof_command_executor.cpp
  src/generic_stages.cpp
  src/policy_runner.cpp
  src/policy_node.cpp
  src/policy_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_DEPENDS})
# ONNX Runtime is an implementation detail: it does not appear in the public headers.
target_link_libraries(${PROJECT_NAME}
  PUBLIC "${cpp_typesupport_target}"
  PRIVATE onnxruntime::onnxruntime)

add_executable(policy_node src/policy_node_main.cpp)
target_link_libraries(policy_node ${PROJECT_NAME})

# All plugin base classes live in this package, so every plugin type is exported against it.
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS policy_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(TEST_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/models")

  ament_add_gtest(test_data_store test/test_data_store.cpp)
  target_link_libraries(test_data_store ${PROJECT_NAME})
  ament_add_gtest(test_policy_pipeline test/test_policy_pipeline.cpp)
  target_link_libraries(test_policy_pipeline ${PROJECT_NAME})
  ament_add_gtest(test_topic_sources test/test_topic_sources.cpp)
  target_link_libraries(test_topic_sources ${PROJECT_NAME})
  ament_add_gtest(test_multi_dof_command_executor test/test_multi_dof_command_executor.cpp)
  target_link_libraries(test_multi_dof_command_executor ${PROJECT_NAME})
  ament_add_gtest(test_observation_term test/test_observation_term.cpp)
  target_link_libraries(test_observation_term ${PROJECT_NAME})
  ament_add_gtest(test_generic_stages test/test_generic_stages.cpp)
  target_link_libraries(test_generic_stages ${PROJECT_NAME})
  foreach(test IN ITEMS test_onnx_inference test_policy_runner test_policy_controller test_policy_node)
    ament_add_gtest(${test} test/${test}.cpp)
    target_link_libraries(${test} ${PROJECT_NAME})
    target_compile_definitions(${test} PRIVATE TEST_MODELS_DIR="${TEST_MODELS_DIR}")
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime ${THIS_PACKAGE_DEPENDS})
ament_package()
