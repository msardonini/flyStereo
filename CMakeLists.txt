cmake_minimum_required(VERSION 3.16)

# Need to include depth-ai before 'project' because it uses Hunter to manage it's packages
# add_subdirectory(external/depthai-core EXCLUDE_FROM_ALL)

project(flyStereo VERSION 0.1.0 LANGUAGES CXX CUDA)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


option(WITH_VPI ON)
option(FLYSTEREO_WITH_VIZ "Compile with viz tools, requires GTK and opencv with viz" OFF)
option(FLYSTEREO_PLOT "Plot the images as the program runs" OFF)
option(COVERAGE "Compile with code coverage" OFF)
option(VERBOSE "Compile with verbosity" OFF)
option(ENABLE_TESTING "Compile unit tests" OFF)
option(FLYSTEREO_BUILD_SENSOR_IO "Build sensor IO" ON)
option(FLYSTEREO_BUILD_ALGORITHMS "Build algorithms" ON)
option(FLYSTEREO_BUILD_APPS "Build applications" ON)

if (VERBOSE)
  set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

if (COVERAGE)
  SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS}")
	SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage ${CMAKE_C_FLAGS}")
endif()

add_subdirectory(external)

add_subdirectory(lib)

if (FLYSTEREO_BUILD_APPS)
  add_subdirectory(apps)
endif()
