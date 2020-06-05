cmake_minimum_required(VERSION 3.10.2)


add_subdirectory(lib)


add_executable(fly_stereo
  src/main.cc
)

target_link_libraries(fly_stereo
  PRIVATE
    fly_stereo_lib
)

target_include_directories(fly_stereo
	PRIVATE
  	${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

