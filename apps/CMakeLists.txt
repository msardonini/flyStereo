cmake_minimum_required(VERSION 3.10.2)

add_subdirectory(acquire_calib_images)

add_subdirectory(flyStereo)

add_subdirectory(imu_compensation)


if(${FLYSTEREO_WITH_VIZ})
  add_subdirectory(viz_demo)
endif()

add_executable(test_time_log
  test_time_log.cc
)

target_link_libraries(test_time_log
  fly_stereo_lib
)
