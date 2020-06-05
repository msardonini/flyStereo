
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)

add_library(fly_stereo_lib
  src/image_processor.cc
  src/mavlink_reader.cc
  src/camera_trigger.cc
  src/camera.cc
)

target_include_directories(fly_stereo_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(fly_stereo_lib
  PRIVATE
    pthread
    Eigen3::Eigen
  PUBLIC
    ${OpenCV_LIBS}
    yaml-cpp
)