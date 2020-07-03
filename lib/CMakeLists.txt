cmake_minimum_required(VERSION 3.10.2)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(SPQR)

add_library(fly_stereo_lib
  src/image_processor.cc
  src/mavlink_reader.cc
  src/camera_trigger.cc
  src/camera.cc
  src/msckf_vio/msckf_vio.cpp
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
)
