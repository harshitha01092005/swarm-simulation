cmake_minimum_required(VERSION 3.16)
project(swarm_fleet_system VERSION 1.0.0 LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(swarm_fleet_system SHARED FleetSystem.cc)
target_compile_features(swarm_fleet_system PRIVATE cxx_std_17)
target_compile_options(swarm_fleet_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(swarm_fleet_system PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
install(TARGETS swarm_fleet_system LIBRARY DESTINATION lib/drone_swarm)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_fleet_protocol tests/test_protocol.cc)
  target_compile_features(test_fleet_protocol PRIVATE cxx_std_17)
  target_include_directories(test_fleet_protocol PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(test_fleet_protocol PRIVATE gz-sim8::gz-sim8 gz-msgs10::gz-msgs10)
  add_test(NAME fleet_protocol COMMAND test_fleet_protocol)
  add_executable(test_fleet_colors tests/test_colors.cc)
  target_compile_features(test_fleet_colors PRIVATE cxx_std_17)
  target_include_directories(test_fleet_colors PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(test_fleet_colors PRIVATE gz-sim8::gz-sim8 gz-msgs10::gz-msgs10)
  add_test(NAME fleet_colors COMMAND test_fleet_colors)
endif()
