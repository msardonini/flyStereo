
# Build Tests
add_executable(fly_stereo_tests
  umat/umat_test_helpers.cc
  umat/umat_vpiimage_tests.cc
  # opt_flow_tests.cc
  umat/umat_tests.cc
  # image_processor_tests.cc
)

target_link_libraries(fly_stereo_tests
  fly_stereo_lib
  gtest
  gtest_main
)


if (COVERAGE)
  SETUP_TARGET_FOR_COVERAGE(
    fly_stereo_coverage  # Name for custom target.
    fly_stereo_tests         # Name of the test driver executable that runs the tests.
    coverage            # Name of output directory.
)
endif()

# Build Tests
add_executable(fly_stereo_benchmark
  benchmark/opt_flow_benchmark.cc
)

target_link_libraries(fly_stereo_benchmark
  fly_stereo_lib
  benchmark::benchmark
)
