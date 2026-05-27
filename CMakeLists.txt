cmake_minimum_required(VERSION 3.20)

project(gripper_refactor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(gripper_core OBJECT
  src/config/config_loader.cpp
  src/utils/math_utils.cpp
  src/utils/geometry_utils.cpp
  src/utils/interpolation.cpp
  src/utils/filter_utils.cpp
  src/hardware_interface/virtual_encoder.cpp
  src/hardware_interface/simulated/simulated_motor.cpp
  src/hardware_interface/damiao/damiao_motor.cpp
  src/hardware_interface/damiao/damiao_protocol.cpp
  src/hardware_interface/damiao/dm_usb2fdcan_transport.cpp
  src/controller/gripper_controller.cpp
  src/controller/nut_position_encoder/nut_position_encoder.cpp
  src/controller/state_machine/gripper_state_machine.cpp
  src/controller/state_machine/self_check_state.cpp
  src/controller/self_check/structure_parameter_identifier.cpp
  src/controller/self_check/friction_identifier.cpp
  src/controller/self_check/friction_anomaly_map.cpp
  src/controller/self_check/travel_limit_identifier.cpp
  src/controller/self_check/motion_health_checker.cpp
  src/controller/self_check/self_check_manager.cpp
  src/controller/safety/safety_limiter.cpp
  src/controller/safety/contact_jam_detector.cpp
  src/controller/safety/debug_constraint_policy.cpp
  src/controller/mechanism/gripper_kinematics.cpp
  src/controller/calibration/force_mapper.cpp
)

target_include_directories(gripper_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(gripper_core PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(gripper_core PRIVATE /W4)
else()
  target_compile_options(gripper_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(gripper_app_support OBJECT
  src/commander/command_dispatcher.cpp
  src/commander/command_line.cpp
  src/ui/runtime_log_model.cpp
  src/ui/ui_controller.cpp
  src/ui/main_window.cpp
  src/ui/web_server.cpp
  src/app/application.cpp
)

target_include_directories(gripper_app_support
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(gripper_app_support PUBLIC cxx_std_17)
target_link_libraries(gripper_app_support PUBLIC gripper_core)

if(MSVC)
  target_compile_options(gripper_app_support PRIVATE /W4)
else()
  target_compile_options(gripper_app_support PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(gripper_app
  src/app/main.cpp
)

target_link_libraries(gripper_app PRIVATE gripper_core gripper_app_support)

if(WIN32)
  target_link_libraries(gripper_app PRIVATE ws2_32)
endif()

add_executable(test_damiao_protocol
  test/test_damiao_protocol.cpp
)
target_link_libraries(test_damiao_protocol PRIVATE gripper_core)
target_include_directories(test_damiao_protocol PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(test_virtual_encoder
  test/test_virtual_encoder.cpp
)
target_link_libraries(test_virtual_encoder PRIVATE gripper_core)
target_include_directories(test_virtual_encoder PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(test_nut_position_encoder
  test/test_nut_position_encoder.cpp
)
target_link_libraries(test_nut_position_encoder PRIVATE gripper_core)
target_include_directories(test_nut_position_encoder PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(test_safety_limiter
  test/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter PRIVATE gripper_core)
target_include_directories(test_safety_limiter PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(test_debug_constraint_policy
  test/test_debug_constraint_policy.cpp
)
target_link_libraries(test_debug_constraint_policy PRIVATE gripper_core)
target_include_directories(test_debug_constraint_policy PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(test_friction_anomaly_map
  test/test_friction_anomaly_map.cpp
)
target_link_libraries(test_friction_anomaly_map PRIVATE gripper_core)
target_include_directories(test_friction_anomaly_map PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(test_state_machine
  test/test_state_machine.cpp
)
target_link_libraries(test_state_machine PRIVATE gripper_core)
target_include_directories(test_state_machine PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(test_motor_bringup
  test/test_motor_bringup.cpp
)
target_link_libraries(test_motor_bringup PRIVATE gripper_core)
target_include_directories(test_motor_bringup PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(test_gripper_workflow
  test/test_gripper_workflow.cpp
)
target_link_libraries(test_gripper_workflow PRIVATE gripper_core)
target_include_directories(test_gripper_workflow PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_executable(test_hardware_selfcheck
  test/test_hardware_selfcheck.cpp
)
target_link_libraries(test_hardware_selfcheck PRIVATE gripper_core)
target_include_directories(test_hardware_selfcheck PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

enable_testing()
add_test(NAME gripper_scripted_demo COMMAND gripper_app --scripted-demo)
add_test(NAME gripper_damiao_placeholder
  COMMAND gripper_app --damiao --scripted-demo)
add_test(NAME test_damiao_protocol COMMAND test_damiao_protocol)
add_test(NAME test_virtual_encoder COMMAND test_virtual_encoder)
add_test(NAME test_nut_position_encoder COMMAND test_nut_position_encoder)
add_test(NAME test_safety_limiter COMMAND test_safety_limiter)
add_test(NAME test_debug_constraint_policy COMMAND test_debug_constraint_policy)
add_test(NAME test_friction_anomaly_map COMMAND test_friction_anomaly_map)
add_test(NAME test_state_machine COMMAND test_state_machine)
add_test(NAME test_motor_bringup COMMAND test_motor_bringup)
add_test(NAME test_gripper_workflow COMMAND test_gripper_workflow)
