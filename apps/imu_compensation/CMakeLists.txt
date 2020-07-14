cmake_minimum_required(VERSION 3.10.2)

add_executable(imu_compensation
  imu_compensation.cc
)

target_link_libraries(imu_compensation
  PRIVATE
    fly_stereo_lib
)
