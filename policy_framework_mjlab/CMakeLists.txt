cmake_minimum_required(VERSION 3.16)
project(policy_framework_mjlab)

# ros2_control's warnings-as-errors set (set_compiler_options()), plus stricter checks below.
find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
# Appended to the set above: virtual-interface mistakes in plugins, unsafe casts, signedness and
# uninitialized reads (most of those are only found in optimized builds).
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Werror=uninitialized -Werror=non-virtual-dtor -Werror=overloaded-virtual
                      -Werror=suggest-override -Werror=implicit-fallthrough -Werror=old-style-cast
                      -Werror=cast-qual -Werror=sign-conversion)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Werror=duplicated-cond -Werror=duplicated-branches -Werror=logical-op)
endif()
add_compile_options(-Werror=uninitialized)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mjlab_preprocessor.cpp
  src/mjlab_postprocessor.cpp
  src/twist_command_term.cpp
  src/extra_inputs.cpp
)
# Not linked by ament_auto (fmt exports only CMake targets); its headers use it too.
target_link_libraries(${PROJECT_NAME} fmt::fmt)
# Parameter declaration and validation, generated from params/*.yaml.
foreach(params IN ITEMS mjlab_preprocessor mjlab_postprocessor)
  generate_parameter_library(${params}_parameters params/${params}_parameters.yaml)
  target_link_libraries(${PROJECT_NAME} ${params}_parameters)
endforeach()
pluginlib_export_plugin_description_file(policy_framework plugins.xml)
# ExtraInput is a base class of this package.
pluginlib_export_plugin_description_file(policy_framework_mjlab extra_input_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # Test-only term plugins (installed and registered only with BUILD_TESTING).
  foreach(term IN ITEMS frame_position_term vector_term)
    add_library(${term} SHARED test/plugins/${term}.cpp)
    target_link_libraries(${term} ${PROJECT_NAME})
    install(TARGETS ${term} DESTINATION lib)
  endforeach()
  pluginlib_export_plugin_description_file(policy_framework test/plugins/test_plugins.xml)
  ament_add_gtest(test_mjlab_preprocessor test/test_mjlab_preprocessor.cpp)
  target_link_libraries(test_mjlab_preprocessor ${PROJECT_NAME})
  target_compile_definitions(test_mjlab_preprocessor PRIVATE
    TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../policy_framework/test/fixtures/urdf/test_robot.urdf")
  ament_add_gtest(test_mjlab_postprocessor test/test_mjlab_postprocessor.cpp)
  target_link_libraries(test_mjlab_postprocessor ${PROJECT_NAME})
  ament_add_gtest(test_twist_command_term test/test_twist_command_term.cpp)
  target_link_libraries(test_twist_command_term ${PROJECT_NAME})
  ament_add_gtest(test_term_max_age test/test_term_max_age.cpp)
  target_link_libraries(test_term_max_age ${PROJECT_NAME})
  target_compile_definitions(test_term_max_age PRIVATE
    TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../policy_framework/test/fixtures/models/sensor.onnx")
  ament_add_gtest(test_shared_command_source test/test_shared_command_source.cpp)
  target_link_libraries(test_shared_command_source ${PROJECT_NAME})
  target_compile_definitions(test_shared_command_source PRIVATE
    TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../policy_framework/test/fixtures/models/command.onnx")
  ament_add_gtest(test_shipped_models test/test_shipped_models.cpp TIMEOUT 300)
  target_link_libraries(test_shipped_models ${PROJECT_NAME})
  ament_add_pytest_test(test_kangaroo_launch test/test_kangaroo_launch.py)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE config launch)
