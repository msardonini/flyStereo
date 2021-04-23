cmake_minimum_required(VERSION 3.10.2)

find_package(OpenCV REQUIRED)

add_executable(viz_demo
  viz_demo.cc
)

target_link_libraries(viz_demo
  PRIVATE
    ${OpenCV_LIBS}
)
