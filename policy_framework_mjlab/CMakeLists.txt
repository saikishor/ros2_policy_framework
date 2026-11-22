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
  src/twist_command_term.cpp
  src/motion_goal_term.cpp
)
# Parameter declaration and validation, generated from params/*.yaml.
foreach(params IN ITEMS mjlab_preprocessor mjlab_postprocessor)
  generate_parameter_library(${params}_parameters params/${params}_parameters.yaml)
  target_link_libraries(${PROJECT_NAME} ${params}_parameters)
endforeach()
pluginlib_export_plugin_description_file(policy_framework plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # Test-only term plugin (installed and registered only with BUILD_TESTING).
  add_library(frame_position_term SHARED test/frame_position_term.cpp)
  target_link_libraries(frame_position_term ${PROJECT_NAME})
  install(TARGETS frame_position_term DESTINATION lib)
  pluginlib_export_plugin_description_file(policy_framework test/test_plugins.xml)
  ament_add_gtest(test_mjlab_preprocessor test/test_mjlab_preprocessor.cpp)
  target_link_libraries(test_mjlab_preprocessor ${PROJECT_NAME})
  target_compile_definitions(test_mjlab_preprocessor PRIVATE
    TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../policy_framework/test/urdf/test_robot.urdf")
  ament_add_gtest(test_mjlab_postprocessor test/test_mjlab_postprocessor.cpp)
  target_link_libraries(test_mjlab_postprocessor ${PROJECT_NAME})
  ament_add_gtest(test_twist_command_term test/test_twist_command_term.cpp)
  target_link_libraries(test_twist_command_term ${PROJECT_NAME})
  ament_add_gtest(test_motion_goal_term test/test_motion_goal_term.cpp)
  target_link_libraries(test_motion_goal_term ${PROJECT_NAME})
  target_compile_definitions(test_motion_goal_term PRIVATE
    TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../policy_framework/test/urdf/test_robot.urdf")
  ament_add_gtest(test_shipped_models test/test_shipped_models.cpp TIMEOUT 300)
  target_link_libraries(test_shipped_models ${PROJECT_NAME})
  ament_add_pytest_test(test_kangaroo_launch test/test_kangaroo_launch.py)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE config launch)
