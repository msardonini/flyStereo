cmake_minimum_required(VERSION 3.10.2)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package (Eigen3 3.3.7 REQUIRED NO_MODULE)


add_executable(fly_stereo
  src/main.cc
  src/image_processor.cc
  src/mavlink_reader.cc
)

target_link_libraries(fly_stereo
  PRIVATE
    yaml-cpp
    ${OpenCV_LIBS}
    pthread
    Eigen3::Eigen
)

target_include_directories(fly_stereo
	PRIVATE
  	${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
  

add_executable(gpio
  src/gpio_main.cc
  )
