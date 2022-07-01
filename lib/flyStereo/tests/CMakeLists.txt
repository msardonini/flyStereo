
# Build Tests
add_executable(fly_stereo_tests
  opt_flow_tests.cc
  image_processor_tests.cc
)

target_link_libraries(fly_stereo_tests
  fly_stereo_lib
  gtest
  gtest_main
)


if (WITH_VPI)
  set(VPI_TEST_SRCS
    umat/umat_vpiimage_tests.cc
    umat/umat_vpiarray_tests.cc)
endif()

 # Build Tests
 add_executable(fly_stereo_umat_tests
 umat/umat_tests.cc
 umat/umat_test_helpers.cc
 ${VPI_TEST_SRCS}
 )

target_link_libraries(fly_stereo_umat_tests
  fly_stereo_lib
  gtest
  gtest_main
)

 # IO tests
 add_executable(fly_stereo_io_tests
 oakd_tests.cc
 )

target_link_libraries(fly_stereo_io_tests
  fly_stereo_lib
  depthai::opencv
  gtest
  gtest_main
)

if (COVERAGE)
  SETUP_TARGET_FOR_COVERAGE(
    fly_stereo_tests_cov  # Name for custom target.
    fly_stereo_tests         # Name of the test driver executable that runs the tests.
    coverage            # Name of output directory.
  )
  SETUP_TARGET_FOR_COVERAGE(
    fly_stereo_umat_tests_cov  # Name for custom target.
    fly_stereo_umat_tests         # Name of the test driver executable that runs the tests.
    coverage            # Name of output directory.
  )

endif()

# # Build Tests
# add_executable(fly_stereo_benchmark
#   benchmark/opt_flow_benchmark.cc
# )

# target_link_libraries(fly_stereo_benchmark
#   fly_stereo_lib
#   benchmark::benchmark
# )
