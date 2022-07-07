cmake_minimum_required(VERSION 3.10.2)


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
    ${CUDA_INCLUDE_DIRS}
    ${GST_INCLUDE_DIRS}
    ${flyStereoRootDir}
)

target_link_libraries(fly_stereo_sensor_io
  PRIVATE
    sqlite3
    pthread
  PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    yaml-cpp
    ${GST_LIBRARIES}
    ${CUDA_LIBRARIES}
    depthai::opencv
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
