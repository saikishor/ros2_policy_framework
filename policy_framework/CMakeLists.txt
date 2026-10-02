cmake_minimum_required(VERSION 3.16)
project(policy_framework)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  srv/SwitchPolicy.srv
  LIBRARY_NAME ${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces rosidl_typesupport_cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/policy_pipeline.cpp
  src/onnx_inference.cpp
  src/topic_sources.cpp
  src/multi_dof_command_executor.cpp
  src/generic_stages.cpp
  src/policy_runner.cpp
  src/policy_node.cpp
  src/policy_controller.cpp
)
target_link_libraries(${PROJECT_NAME} onnxruntime::onnxruntime "${cpp_typesupport_target}")
# All plugin base classes live in this package, so every plugin type is exported against it.
ament_auto_add_executable(policy_node src/policy_node_main.cpp)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_data_store test/test_data_store.cpp)
  target_include_directories(test_data_store PRIVATE include)
  ament_add_gtest(test_policy_pipeline test/test_policy_pipeline.cpp)
  target_link_libraries(test_policy_pipeline ${PROJECT_NAME})
  ament_add_gtest_executable(test_onnx_inference test/test_onnx_inference.cpp)
  target_link_libraries(test_onnx_inference ${PROJECT_NAME})
  target_compile_definitions(test_onnx_inference PRIVATE
    TEST_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/models")
  ament_add_gtest_test(test_onnx_inference)
  ament_add_gtest(test_topic_sources test/test_topic_sources.cpp)
  target_link_libraries(test_topic_sources ${PROJECT_NAME})
  ament_add_gtest(test_multi_dof_command_executor test/test_multi_dof_command_executor.cpp)
  target_link_libraries(test_multi_dof_command_executor ${PROJECT_NAME})
  ament_add_gtest(test_generic_stages test/test_generic_stages.cpp)
  target_link_libraries(test_generic_stages ${PROJECT_NAME})
  ament_add_gtest(test_policy_runner test/test_policy_runner.cpp)
  target_link_libraries(test_policy_runner ${PROJECT_NAME})
  target_compile_definitions(test_policy_runner PRIVATE
    TEST_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/models")
  ament_add_gtest(test_policy_controller test/test_policy_controller.cpp)
  target_link_libraries(test_policy_controller ${PROJECT_NAME})
  target_compile_definitions(test_policy_controller PRIVATE
    TEST_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/models")
  ament_add_gtest(test_policy_node test/test_policy_node.cpp)
  target_link_libraries(test_policy_node ${PROJECT_NAME})
  target_compile_definitions(test_policy_node PRIVATE
    TEST_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/models")
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
