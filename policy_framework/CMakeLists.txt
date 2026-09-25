cmake_minimum_required(VERSION 3.16)
project(policy_framework)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/policy_pipeline.cpp
  src/onnx_inference.cpp
)
target_link_libraries(${PROJECT_NAME} onnxruntime::onnxruntime)
# All plugin base classes live in this package, so every plugin type is exported against it.
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

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
endif()

ament_auto_package()
