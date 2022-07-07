cmake_minimum_required(VERSION 3.10.2)

add_executable(fly_stereo
  fly_stereo.cc
)

target_link_libraries(fly_stereo
  PRIVATE
    fly_stereo_lib
)
