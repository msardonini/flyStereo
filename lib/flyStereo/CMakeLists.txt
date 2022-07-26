cmake_minimum_required(VERSION 3.10.2)

if (FLYSTEREO_BUILD_SENSOR_IO)
  add_subdirectory(sensor_io)
endif()


if (FLYSTEREO_BUILD_ALGORITHMS)
  find_package(OpenCV REQUIRED)
  find_package(SQLite3 REQUIRED)

  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

  if (${WITH_VPI})
    find_package(vpi REQUIRED)
  endif()

  if (${FLYSTEREO_WITH_VIZ})
    # TODO make this it's own module
    set (FLY_STEREO_VIZ_SRC visualization/visualization.cc)
  endif()



  add_library(fly_stereo_lib
    kalman_filter.cc
    image_processing/image_processor.cc
    visualization/draw_to_image.cc
    pipeline.cc
    vio.cc
    ${FLY_STEREO_VIZ_SRC}
  )

  target_include_directories(fly_stereo_lib
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${GST_INCLUDE_DIRS}
      ${CUDA_INCLUDE_DIRS}
      ${FLYSTEREO_SRC_DIR}
      ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  )

  target_link_libraries(fly_stereo_lib
    PRIVATE
      sqlite3
      pthread
    PUBLIC
      ${OpenCV_LIBS}
      ${libLAS_LIBRARIES}
      yaml-cpp
      ${GST_LIBRARIES}
      ${CUDA_LIBRARIES}
      thread_pool
      fly_stereo_sensor_io
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

  if (FLYSTEREO_WITH_VIZ)
    target_compile_definitions(fly_stereo_lib
      PUBLIC
        WITH_VIZ
      )
  endif()

  target_compile_options(fly_stereo_lib
    PRIVATE
      -Wall
  )

  # SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
  # SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

  set_property(TARGET fly_stereo_lib PROPERTY CXX_STANDARD 20)

  if (${ENABLE_TESTING})
    add_subdirectory(tests)
  endif()

  # add_executable(oakd_orb3
  #   oakd_orb3.cc
  # )

  # find_package(Pangolin REQUIRED)

  # target_link_libraries(oakd_orb3
  #   PUBLIC
  #     fly_stereo_lib
  #     depthai::opencv
  #     ${OpenCV_LIBS}
  #     ${PROJECT_SOURCE_DIR}/external/ORB_SLAM3/lib/libORB_SLAM3.so
  #     Eigen3::Eigen
  #     ${Pangolin_LIBRARIES}
  # )

  # target_include_directories(oakd_orb3
  #   PUBLIC
  #     ${PROJECT_SOURCE_DIR}/external/ORB_SLAM3/include
  #     ${PROJECT_SOURCE_DIR}/external/ORB_SLAM3
  #     ${PROJECT_SOURCE_DIR}/external/ORB_SLAM3/Thirdparty/Sophus
  #     ${PROJECT_SOURCE_DIR}/external/ORB_SLAM3/include/CameraModels
  # )

endif()
