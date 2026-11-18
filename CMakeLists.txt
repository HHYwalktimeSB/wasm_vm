cmake_minimum_required(VERSION 3.20)
project(robot_sim LANGUAGES CXX)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Threads REQUIRED)
add_library(robot_sim_core src/bus.cpp src/vm.cpp src/simulation.cpp src/peripheral_bridge.cpp)
target_link_libraries(robot_sim_core PRIVATE Threads::Threads)
target_include_directories(robot_sim_core PUBLIC include)
target_compile_features(robot_sim_core PUBLIC cxx_std_20)
add_executable(robot_sim_cli src/main.cpp src/cli_scenario.cpp)
target_link_libraries(robot_sim_cli PRIVATE robot_sim_core)
include(CTest)
if(BUILD_TESTING)
  add_executable(robot_sim_tests tests/core.cpp)
  target_link_libraries(robot_sim_tests PRIVATE robot_sim_core)
  add_test(NAME core COMMAND robot_sim_tests)
  add_executable(robot_bridge_tests tests/peripheral_bridge.cpp)
  target_link_libraries(robot_bridge_tests PRIVATE robot_sim_core)
  add_test(NAME peripheral_bridge COMMAND robot_bridge_tests)
  add_test(NAME help COMMAND robot_sim_cli --help)
  add_test(NAME cli_replay COMMAND "${CMAKE_COMMAND}"
    "-DCLI=$<TARGET_FILE:robot_sim_cli>" "-DFIXTURE=$<TARGET_FILE:robot_sim_tests>"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/cli-test"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/cli_replay.cmake")
endif()


option(ROBOT_TEST_C_GUEST "Build and test the Clang WASM guest" OFF)
if(BUILD_TESTING AND ROBOT_TEST_C_GUEST)
  find_program(ROBOT_CLANG NAMES clang REQUIRED)
  set(guest "${CMAKE_CURRENT_BINARY_DIR}/controller.wasm")
  add_custom_command(OUTPUT "${guest}"
    COMMAND "${CMAKE_COMMAND}" "-DCLANG=${ROBOT_CLANG}"
      "-DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/examples/controller.c"
      "-DOUTPUT=${guest}" -P "${CMAKE_CURRENT_SOURCE_DIR}/sdk/c/build_guest.cmake"
    DEPENDS examples/controller.c sdk/c/robot_mmio.h sdk/c/build_guest.cmake
    VERBATIM)
  add_custom_target(robot_c_guest ALL DEPENDS "${guest}")
  add_dependencies(robot_bridge_tests robot_c_guest)
  add_test(NAME c_peripheral_bridge COMMAND robot_bridge_tests "${guest}")
  add_test(NAME c_guest COMMAND "${CMAKE_COMMAND}" "-DCLI=$<TARGET_FILE:robot_sim_cli>"
    "-DMODULE=${guest}" -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/c_guest.cmake")
  set(runtime_guest "${CMAKE_CURRENT_BINARY_DIR}/runtime.wasm")
  add_custom_command(OUTPUT "${runtime_guest}"
    COMMAND "${CMAKE_COMMAND}" "-DCLANG=${ROBOT_CLANG}"
      "-DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/examples/runtime.c"
      "-DOUTPUT=${runtime_guest}" -P "${CMAKE_CURRENT_SOURCE_DIR}/sdk/c/build_guest.cmake"
    DEPENDS examples/runtime.c sdk/c/robot_mmio.h sdk/c/build_guest.cmake
    VERBATIM)
  add_custom_target(robot_c_runtime ALL DEPENDS "${runtime_guest}")
  add_test(NAME c_runtime COMMAND robot_sim_cli test "${runtime_guest}" --ticks 2
    --ram-pages 2 --expect-status 2 --expect-commands 1)
endif()
