cmake_minimum_required(VERSION 3.16)
project(policy_framework)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/policy_pipeline.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_data_store test/test_data_store.cpp)
  target_include_directories(test_data_store PRIVATE include)
  ament_add_gtest(test_policy_pipeline test/test_policy_pipeline.cpp)
  target_link_libraries(test_policy_pipeline ${PROJECT_NAME})
endif()

ament_auto_package()
