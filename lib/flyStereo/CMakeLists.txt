cmake_minimum_required(VERSION 3.10.2)

find_package(OpenCV REQUIRED)
find_package(SQLite3)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if (${WITH_VPI})
  find_package(vpi REQUIRED)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.4
  gstreamer-video-1.0>=1.4
  gstreamer-app-1.0>=1.4)

if (${FLYSTEREO_WITH_VIZ})
  # TODO make this it's own module
  set (FLY_STEREO_VIZ_SRC visualization/visualization.cc)
endif()


add_library(fly_stereo_lib
  kalman_filter.cc
  # sql_logger.cc
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
    ${OpenCV_LIBS}
    ${libLAS_LIBRARIES}
    yaml-cpp
    ${GST_LIBRARIES}
    ${CUDA_LIBRARIES}
    thread_pool
    depthai::opencv
)

if (${WITH_VPI})
  target_compile_definitions(fly_stereo_lib
    PUBLIC
      WITH_VPI
  )
  target_link_libraries(fly_stereo_lib
    PUBLIC
      vpi
  )
endif()

if (FLYSTEREO_PLOT)
  target_compile_definitions(fly_stereo_lib
    PUBLIC
      FLYSTEREO_VISUALIZE_DEBUG_VIDEO
    )
endif()

target_compile_options(fly_stereo_lib
  PRIVATE
    -Wall
)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

#target_compile_definitions(fly_stereo_lib
#  PUBLIC
#    -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#    -DEIGEN_DONT_VECTORIZE
#)

set_property(TARGET fly_stereo_lib PROPERTY CXX_STANDARD 20)

if (ENABLE_TESTING)
  add_subdirectory(tests)
endif()
