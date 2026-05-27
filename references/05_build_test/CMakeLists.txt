cmake_minimum_required(VERSION 3.16)

project(gripper_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.venv/Scripts/python.exe")
  set(Python3_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/.venv/Scripts/python.exe"
      CACHE FILEPATH "Python interpreter" FORCE)
endif()
find_package(Python3 COMPONENTS Interpreter)

add_library(gripper_control_objects OBJECT
  src/gripper_control/contact_jam_detector.cpp
  src/gripper_control/damiao/dm_j4310_motor.cpp
  src/gripper_control/damiao/dm_j4310_protocol.cpp
  src/gripper_control/force_command_mapper.cpp
  src/gripper_control/friction_identifier.cpp
  src/gripper_control/gripper_controller.cpp
  src/gripper_control/safety_limiter.cpp
  src/gripper_control/simulated_motor.cpp
  src/gripper_control/types.cpp
)

target_include_directories(gripper_control_objects
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(gripper_control_demo
  demo/gripper_control_demo.cpp
)

target_link_libraries(gripper_control_demo
  PRIVATE
    gripper_control_objects
)

add_executable(gripper_control_unit_tests
  test/gripper_control_unit_tests.cpp
)

target_link_libraries(gripper_control_unit_tests
  PRIVATE
    gripper_control_objects
)

if(Python3_Interpreter_FOUND)
  add_custom_target(gripper_control_ui_bundle
    COMMAND ${Python3_EXECUTABLE} -m PyInstaller
            --noconfirm
            --clean
            --windowed
            --name gripper_control_ui
            --distpath ${CMAKE_BINARY_DIR}/ui/dist
            --workpath ${CMAKE_BINARY_DIR}/ui/build
            --specpath ${CMAKE_BINARY_DIR}/ui
            --paths ${CMAKE_CURRENT_SOURCE_DIR}/src
            --hidden-import can.interfaces.slcan
            --hidden-import can.interfaces.pcan
            --hidden-import serial
            --hidden-import serial.tools.list_ports
            --add-binary "${CMAKE_CURRENT_SOURCE_DIR}/dlls/*.dll;dlls"
            ${CMAKE_CURRENT_SOURCE_DIR}/src/gripper_control/ui/gripper_control_ui.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building gripper_control_ui Windows executable"
    VERBATIM
  )
endif()
