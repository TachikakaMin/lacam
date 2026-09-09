# PROTECTED REGRESSION TEST: the JNA shared library must configure and build
# without discovering pkg-config or yaml-cpp on the host.
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED REPO_ROOT OR NOT IS_DIRECTORY "${REPO_ROOT}")
  message(FATAL_ERROR "REPO_ROOT must name the dd-lacam source directory")
endif()
if(NOT DEFINED BUILD_DIR OR BUILD_DIR STREQUAL "")
  message(FATAL_ERROR "BUILD_DIR must name an isolated build directory")
endif()

execute_process(
  COMMAND
    "${CMAKE_COMMAND}"
    -S "${REPO_ROOT}"
    -B "${BUILD_DIR}"
    -DCMAKE_BUILD_TYPE=Release
    -DCARRIER_LACAM_PORTABLE_ONLY=ON
    -DCMAKE_DISABLE_FIND_PACKAGE_PkgConfig=TRUE
    -DCMAKE_DISABLE_FIND_PACKAGE_yaml-cpp=TRUE
    -DCMAKE_DISABLE_FIND_PACKAGE_YAML_CPP=TRUE
  RESULT_VARIABLE configure_result
  OUTPUT_VARIABLE configure_stdout
  ERROR_VARIABLE configure_stderr)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR
    "portable configure failed without yaml-cpp:\n"
    "${configure_stdout}\n${configure_stderr}")
endif()

execute_process(
  COMMAND
    "${CMAKE_COMMAND}" --build "${BUILD_DIR}"
    --target carrier_lacam --parallel 2
  RESULT_VARIABLE build_result
  OUTPUT_VARIABLE build_stdout
  ERROR_VARIABLE build_stderr)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR
    "portable carrier_lacam build failed:\n"
    "${build_stdout}\n${build_stderr}")
endif()

file(
  GLOB_RECURSE shared_libraries
  LIST_DIRECTORIES FALSE
  "${BUILD_DIR}/*carrier_lacam*.so"
  "${BUILD_DIR}/*carrier_lacam*.dylib"
  "${BUILD_DIR}/*carrier_lacam*.dll")
if(shared_libraries STREQUAL "")
  message(FATAL_ERROR
    "carrier_lacam target built without producing a shared library")
endif()
