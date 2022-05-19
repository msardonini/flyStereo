cmake_minimum_required(VERSION 3.10.2)

find_package(OpenCV REQUIRED)


find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if (${WITH_VPI})
  find_package(vpi REQUIRED)
endif()

find_package(PkgConfig)
pkg_check_modules(GST gstreamer-1.0>=1.4
  gstreamer-video-1.0>=1.4
  gstreamer-app-1.0>=1.4)

if(NOT GST_FOUND)
  message(FATAL_ERROR "Did not find gstreamer libraries, try: \n
  ' $ sudo apt-get install libgstreamer-plugins-base1.0-dev' ")
endif()

find_package(benchmark REQUIRED)

if (${FLYSTEREO_WITH_VIZ})
  # TODO make this it's own module
  set (FLY_STEREO_VIZ_SRC visualization/visualization.cc)
endif()


#add_definitions(-march=native)
add_library(fly_stereo_lib
  image_processing/image_processor.cc
  vio.cc
  kalman_filter.cc
  sql_logger.cc
  sensor_io/sensor_interface.cc
  sensor_io/mavlink_reader.cc
  sensor_io/camera_trigger.cc
  sensor_io/camera.cc
  sensor_io/image_sink.cc
  visualization/draw_to_image.cc
  pipeline.cc
  ${FLY_STEREO_VIZ_SRC}
)

target_include_directories(fly_stereo_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SPQR_INCLUDES}
    ${GST_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${flyStereoRootDir}
)

target_link_libraries(fly_stereo_lib
  PRIVATE
    sqlite3
    pthread
  PUBLIC
    Eigen3::Eigen
    ${SPQR_LIBRARIES}
    ${OpenCV_LIBS}
    ${libLAS_LIBRARIES}
    yaml-cpp
    ${GST_LIBRARIES}
    ${CUDA_LIBRARIES}
)

if (${WITH_VPI})
 target_compile_definitions(fly_stereo_lib
  PUBLIC
    WITH_VPI)
  target_link_libraries(fly_stereo_lib
    PUBLIC
      vpi)
  endif()

target_compile_options(fly_stereo_lib
  PRIVATE
    -Wall
    -pg
)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

#target_compile_definitions(fly_stereo_lib
#  PUBLIC
#    -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#    -DEIGEN_DONT_VECTORIZE
#)

set_property(TARGET fly_stereo_lib PROPERTY CXX_STANDARD 20)


add_subdirectory(tests)
