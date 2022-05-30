cmake_minimum_required(VERSION 3.10.2)

project(flyStereo VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


option(WITH_VPI ON)
option(FLYSTEREO_WITH_VIZ "Compile with viz tools, requires GTK and opencv with viz" OFF)
option(COVERAGE "Compile with code coverage" OFF)
option(VERBOSE "Compile with verbosity" OFF)

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

add_subdirectory(apps)
