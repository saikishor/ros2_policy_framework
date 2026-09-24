cmake_minimum_required(VERSION 3.16)
project(policy_framework)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_data_store test/test_data_store.cpp)
  target_include_directories(test_data_store PRIVATE include)
endif()

ament_auto_package()
