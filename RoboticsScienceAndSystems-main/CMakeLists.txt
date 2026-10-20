cmake_minimum_required(VERSION 3.16)
project(stampc3_tree_host_checks LANGUAGES CXX)

# These are desktop checks only. PlatformIO continues to build the firmware.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(TREE1_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../first_tree" CACHE PATH
    "Original first_tree checkout used as an independent geometry reference")
get_filename_component(TREE1_SOURCE_DIR "${TREE1_SOURCE_DIR}" ABSOLUTE)
if(NOT EXISTS "${TREE1_SOURCE_DIR}/cpp/tree1_runtime.hpp")
  message(FATAL_ERROR "Set TREE1_SOURCE_DIR to the original first_tree checkout")
endif()

set(STAMP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/StampC3_Template")
add_library(tree_adapters OBJECT
  "${STAMP_DIR}/Tree1PathController.cpp"
  "${STAMP_DIR}/Tree2MotorController.cpp"
  "${STAMP_DIR}/RouteCommandParser.cpp")
target_include_directories(tree_adapters PUBLIC "${STAMP_DIR}")
# Compile the actual embedded adapters without C++ exception support, like ESP32.
if(MSVC)
  target_compile_options(tree_adapters PRIVATE /W4 /EHs-c-)
else()
  target_compile_options(tree_adapters PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions)
endif()

add_executable(tree_models_tests tests/tree_models_tests.cpp)
target_link_libraries(tree_models_tests PRIVATE tree_adapters)
target_compile_definitions(tree_models_tests PRIVATE
  TREE1_REFERENCE_RUNTIME="${TREE1_SOURCE_DIR}/cpp/tree1_runtime.hpp")
if(MSVC)
  target_compile_options(tree_models_tests PRIVATE /W4)
else()
  target_compile_options(tree_models_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_test(NAME embedded_tree_models COMMAND tree_models_tests)

add_executable(controller_cycle_tests tests/controller_cycle_tests.cpp
  "${STAMP_DIR}/OdometryModel.cpp" "${STAMP_DIR}/TaskTimer.cpp")
target_include_directories(controller_cycle_tests PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/arduino_stubs")
target_link_libraries(controller_cycle_tests PRIVATE tree_adapters)
add_test(NAME controller_cycles COMMAND controller_cycle_tests)

add_executable(route_command_tests tests/route_command_tests.cpp)
target_link_libraries(route_command_tests PRIVATE tree_adapters)
add_test(NAME route_commands COMMAND route_command_tests)

add_executable(processing_parity tests/processing_parity.cpp)
target_link_libraries(processing_parity PRIVATE tree_adapters)
