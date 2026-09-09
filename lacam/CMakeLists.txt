cmake_minimum_required(VERSION 3.16)
project(lacam)

option(
  CARRIER_LACAM_BUILD_NATIVE
  "Build the YAML-backed native lacam archive"
  ON)

file(
  GLOB ALL_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
set(
  YAML_SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/dd_carrier_yaml.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/instance_yaml.cpp")
set(CORE_SRCS ${ALL_SRCS})
list(REMOVE_ITEM CORE_SRCS ${YAML_SRCS})

add_library(
  carrier_lacam_api_objects OBJECT
  "${CMAKE_CURRENT_SOURCE_DIR}/interface/carrier_lacam_jna.cpp")
set_target_properties(
  carrier_lacam_api_objects
  PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(
  carrier_lacam_api_objects PRIVATE -O3 -Wall)
target_compile_features(
  carrier_lacam_api_objects PUBLIC cxx_std_17)
target_include_directories(
  carrier_lacam_api_objects
  PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/interface")

# Existing CLI/tests keep the historical native-optimized static target.
# The C ABI stays out of this archive so consumers cannot accidentally link a
# second copy alongside libcarrier_lacam.so.
if(CARRIER_LACAM_BUILD_NATIVE)
  add_library(
    ${PROJECT_NAME} STATIC
    ${CORE_SRCS}
    ${YAML_SRCS})
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
  target_compile_options(
    ${PROJECT_NAME} PUBLIC
    -O3 -Wall -mtune=native -march=native)
  target_compile_features(
    ${PROJECT_NAME} PUBLIC cxx_std_17)
  target_include_directories(
    ${PROJECT_NAME} INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_include_directories(
    ${PROJECT_NAME} PRIVATE
    ${YAML_CPP_INCLUDE_DIRS})
  target_link_directories(
    ${PROJECT_NAME} PUBLIC
    ${YAML_CPP_LIBRARY_DIRS})
  target_link_libraries(
    ${PROJECT_NAME} PUBLIC
    ${YAML_CPP_LIBRARIES})
endif()

# Code-Labyrinth loads this portable, YAML-free target through JNA. It
# compiles the same production sources and invokes the same controller; only
# the build boundary differs from the native-optimized command-line target.
add_library(carrier_lacam_core STATIC ${CORE_SRCS})
set_target_properties(
  carrier_lacam_core
  PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(
  carrier_lacam_core PRIVATE -O3 -Wall)
target_compile_features(
  carrier_lacam_core PUBLIC cxx_std_17)
target_include_directories(
  carrier_lacam_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(
  carrier_lacam SHARED
  $<TARGET_OBJECTS:carrier_lacam_api_objects>)
target_compile_features(
  carrier_lacam PUBLIC cxx_std_17)
target_include_directories(
  carrier_lacam PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/interface")
target_link_libraries(
  carrier_lacam PRIVATE carrier_lacam_core)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
   CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
  if(CARRIER_LACAM_BUILD_NATIVE)
    target_link_libraries(${PROJECT_NAME} PUBLIC stdc++fs)
  endif()
  target_link_libraries(carrier_lacam_core PUBLIC stdc++fs)
endif()
