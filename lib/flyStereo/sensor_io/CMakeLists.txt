cmake_minimum_required(VERSION 3.10.2)

if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

find_package(depthai CONFIG REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.4
  gstreamer-video-1.0>=1.4
  gstreamer-app-1.0>=1.4
)

set(CMAKE_VERBOSE_MAKEFILE ON)




add_library(fly_stereo_sensor_io
  mavlink_reader.cc
  camera_trigger.cc
  camera.cc
  oakd.cc
  sql_src.cc
  arducam_system.cc
  sql_sink.cc
  image_sink.cc
)


target_include_directories(fly_stereo_sensor_io
  PUBLIC
    $<INSTALL_INTERFACE:${CUDA_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:${GST_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${FLYSTEREO_SRC_DIR}>
    $<BUILD_INTERFACE:${CUDA_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${GST_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}>
  )

target_link_libraries(fly_stereo_sensor_io
  PRIVATE
    sqlite3
    pthread
  PUBLIC
    spdlog::spdlog
    depthai::opencv
    ${OpenCV_LIBS}
    yaml-cpp
    ${GST_LIBRARIES}
    ${CUDA_LIBRARIES}
)

if (${WITH_VPI})
  target_compile_definitions(fly_stereo_sensor_io
    PUBLIC
      WITH_VPI
  )
  target_link_libraries(fly_stereo_sensor_io
    PUBLIC
      vpi
  )
endif()

target_compile_options(fly_stereo_sensor_io
  PRIVATE
    -Wall
)

# install(TARGETS fly_stereo_sensor_io
# LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
# ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
# )

# install(
#   DIRECTORY ${FLYSTEREO_SRC_DIR}
#   DESTINATION /usr/local/include
# )
