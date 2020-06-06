cmake_minimum_required(VERSION 3.10.2)

add_executable(acquire_calib_images
  acquire_calib_images.cc
)

target_link_libraries(acquire_calib_images
  PRIVATE
    fly_stereo_lib
    stdc++fs
)
