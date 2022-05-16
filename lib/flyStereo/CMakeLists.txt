cmake_minimum_required(VERSION 3.10.2)

# find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_PREFIX_PATH /usr/local/lib/cmake/opengv-1.0)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# find_package(libLAS)

find_package(vpi REQUIRED)
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
    vpi
)

target_compile_options(fly_stereo_lib
  PRIVATE
    -Wall
)

#target_compile_definitions(fly_stereo_lib
#  PUBLIC
#    -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#    -DEIGEN_DONT_VECTORIZE
#)

set_property(TARGET fly_stereo_lib PROPERTY CXX_STANDARD 20)



# Build Tests
add_executable(fly_stereo_tests
  test/opt_flow_tests.cc
  test/image_processor_tests.cc
  # test/umat_tests.cc
)

target_link_libraries(fly_stereo_tests
  fly_stereo_lib
  gtest
  gtest_main
)

# Build Tests
add_executable(fly_stereo_benchmark
  test/benchmark/opt_flow_benchmark.cc
)

target_link_libraries(fly_stereo_benchmark
  fly_stereo_lib
  benchmark::benchmark
)