cmake_minimum_required(VERSION 3.10.2)

# find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_PREFIX_PATH /usr/local/lib/cmake/opengv-1.0)
find_package(opengv REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(SPQR)

add_definitions(-march=native)
add_library(fly_stereo_lib
  src/image_processor.cc
  src/sensor_io/sensor_interface.cc
  src/sensor_io/mavlink_reader.cc
  src/sensor_io/camera_trigger.cc
  src/sensor_io/camera.cc
)

target_include_directories(fly_stereo_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SPQR_INCLUDES}
)

target_link_libraries(fly_stereo_lib
  PRIVATE
    pthread
  PUBLIC
    Eigen3::Eigen
    ${SPQR_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    opengv
)
