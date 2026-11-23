cmake_minimum_required(VERSION 3.16)
project(policy_framework)

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

set(THIS_PACKAGE_DEPENDS
  control_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  lifecycle_msgs
  nav_msgs
  pal_statistics
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
# Prints a stack trace on crashes of every executable built here (policy_node, tests).
find_package(backward_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(onnxruntime_vendor REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  msg/MockFault.msg
  srv/SwitchPolicy.srv
  LIBRARY_NAME ${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces rosidl_typesupport_cpp)
# After rosidl_generate_interfaces: Pinocchio's config (through eigenpy) sets up Python in a way
# that makes rosidl's PythonInterp lookup fail on a fresh configure.
find_package(pinocchio REQUIRED)

# Parameter declaration and validation, generated from params/**/*.yaml (header-only targets,
# named after the file: params/hosts/policy_node_parameters.yaml -> policy_node_parameters).
set(PARAMETER_LIBRARIES)
foreach(path IN ITEMS
    params/core/kinematics_parameters.yaml
    params/hosts/policy_controller_parameters.yaml
    params/hosts/policy_node_parameters.yaml
    params/hosts/policy_runner_parameters.yaml
    params/plugins/executors/multi_dof_command_executor_parameters.yaml
    params/plugins/executors/pd_effort_executor_parameters.yaml
    params/plugins/executors/reference_interface_executor_parameters.yaml
    params/plugins/inference/onnx_inference_parameters.yaml
    params/plugins/safety/jump_check_parameters.yaml
    params/plugins/safety/mock_safety_check_parameters.yaml
    params/plugins/safety/state_bounds_check_parameters.yaml
    params/plugins/sources/imu_source_parameters.yaml
    params/plugins/sources/joint_state_source_parameters.yaml
    params/plugins/sources/topic_parameters.yaml
    params/plugins/sources/twist_source_parameters.yaml
    params/plugins/stages/concat_preprocessor_parameters.yaml
    params/plugins/stages/split_postprocessor_parameters.yaml
  )
  get_filename_component(params ${path} NAME_WE)
  generate_parameter_library(${params} ${path})
  list(APPEND PARAMETER_LIBRARIES ${params})
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/core/policy_pipeline.cpp
  src/core/kinematics.cpp
  src/plugins/safety/safety_checks.cpp
  src/core/safety_monitor.cpp
  src/plugins/inference/onnx_inference.cpp
  src/plugins/sources/topic_sources.cpp
  src/plugins/executors/multi_dof_command_executor.cpp
  src/plugins/stages/generic_stages.cpp
  src/plugins/executors/interface_executors.cpp
  src/hosts/policy_runner.cpp
  src/hosts/policy_node.cpp
  src/hosts/policy_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Internal headers shared by the sources (not installed).
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_DEPENDS})
# ONNX Runtime and Pinocchio are implementation details: they do not appear in the public
# headers.
target_link_libraries(${PROJECT_NAME}
  PUBLIC "${cpp_typesupport_target}" Eigen3::Eigen
  PRIVATE onnxruntime::onnxruntime pinocchio::pinocchio ${PARAMETER_LIBRARIES})

add_executable(policy_node src/hosts/policy_node_main.cpp)
target_link_libraries(policy_node ${PROJECT_NAME} policy_node_parameters)

# All plugin base classes live in this package, so every plugin type is exported against it.
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS policy_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(TEST_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/models")

  ament_add_gtest(test_data_store test/core/test_data_store.cpp)
  target_link_libraries(test_data_store ${PROJECT_NAME})
  ament_add_gtest(test_safety_monitor test/core/test_safety_monitor.cpp)
  target_link_libraries(test_safety_monitor ${PROJECT_NAME})
  target_compile_definitions(test_safety_monitor PRIVATE
    TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/urdf/test_robot.urdf")
  ament_add_gtest(test_safety_checks test/plugins/safety/test_safety_checks.cpp)
  target_link_libraries(test_safety_checks ${PROJECT_NAME})
  ament_add_gtest(test_policy_pipeline test/core/test_policy_pipeline.cpp)
  target_link_libraries(test_policy_pipeline ${PROJECT_NAME})
  ament_add_gtest(test_topic_sources test/plugins/sources/test_topic_sources.cpp)
  target_link_libraries(test_topic_sources ${PROJECT_NAME})
  ament_add_gtest(test_multi_dof_command_executor test/plugins/executors/test_multi_dof_command_executor.cpp)
  target_link_libraries(test_multi_dof_command_executor ${PROJECT_NAME})
  ament_add_gtest(test_observation_term test/interfaces/test_observation_term.cpp)
  target_link_libraries(test_observation_term ${PROJECT_NAME})
  ament_add_gtest(test_interface_executors test/plugins/executors/test_interface_executors.cpp)
  target_link_libraries(test_interface_executors ${PROJECT_NAME})
  ament_add_gtest(test_kinematics test/core/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME})
  target_compile_definitions(test_kinematics PRIVATE
    TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/urdf/test_robot.urdf")
  ament_add_gtest(test_generic_stages test/plugins/stages/test_generic_stages.cpp)
  target_link_libraries(test_generic_stages ${PROJECT_NAME})
  foreach(path IN ITEMS
      test/plugins/inference/test_onnx_inference.cpp
      test/hosts/test_policy_runner.cpp
      test/hosts/test_policy_controller.cpp
      test/hosts/test_policy_node.cpp)
    get_filename_component(test ${path} NAME_WE)
    ament_add_gtest(${test} ${path})
    target_link_libraries(${test} ${PROJECT_NAME})
    target_compile_definitions(${test} PRIVATE TEST_MODELS_DIR="${TEST_MODELS_DIR}"
      TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/urdf/test_robot.urdf")
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime Eigen3 ${THIS_PACKAGE_DEPENDS})
ament_package()
