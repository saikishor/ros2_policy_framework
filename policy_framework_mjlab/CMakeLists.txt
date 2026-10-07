cmake_minimum_required(VERSION 3.16)
project(policy_framework_mjlab)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mjlab_preprocessor.cpp
  src/mjlab_postprocessor.cpp
)
pluginlib_export_plugin_description_file(policy_framework plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mjlab_preprocessor test/test_mjlab_preprocessor.cpp)
  target_link_libraries(test_mjlab_preprocessor ${PROJECT_NAME})
  ament_add_gtest(test_mjlab_postprocessor test/test_mjlab_postprocessor.cpp)
  target_link_libraries(test_mjlab_postprocessor ${PROJECT_NAME})
  ament_add_gtest(test_shipped_models test/test_shipped_models.cpp TIMEOUT 300)
  target_link_libraries(test_shipped_models ${PROJECT_NAME})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE config launch)
