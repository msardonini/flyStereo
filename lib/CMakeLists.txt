cmake_minimum_required(VERSION 3.10.2)

# find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_PREFIX_PATH /usr/local/lib/cmake/opengv-1.0)
find_package(opengv REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(libLAS)

find_package(PkgConfig)
pkg_check_modules(GST gstreamer-1.0>=1.4
  gstreamer-video-1.0>=1.4
  gstreamer-app-1.0>=1.4)

if(NOT GST_FOUND)
  message(FATAL_ERROR "Did not find gstreamer libraries, try: \n
  ' $ sudo apt-get install libgstreamer-plugins-base1.0-dev' ")
endif()



add_definitions(-march=native)
add_library(fly_stereo_lib
  src/image_processor.cc
  src/vio.cc
  src/kalman_filter.cc
  src/sensor_io/sensor_interface.cc
  src/sensor_io/mavlink_reader.cc
  src/sensor_io/camera_trigger.cc
  src/sensor_io/camera.cc
)

target_include_directories(fly_stereo_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SPQR_INCLUDES}
    ${GST_INCLUDE_DIRS}
)

target_link_libraries(fly_stereo_lib
  PRIVATE
    pthread
  PUBLIC
    Eigen3::Eigen
    ${SPQR_LIBRARIES}
    ${OpenCV_LIBS}
    ${libLAS_LIBRARIES}
    yaml-cpp
    opengv
    ${GST_LIBRARIES}
)

target_compile_definitions(fly_stereo_lib
  PUBLIC
    -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
    -DEIGEN_DONT_VECTORIZE
)

set_property(TARGET fly_stereo_lib PROPERTY CXX_STANDARD 17)
