
# Build Tests
add_executable(fly_stereo_tests
  # opt_flow_tests.cc
  # umat_tests.cc
  image_processor_tests.cc
)

target_link_libraries(fly_stereo_tests
  fly_stereo_lib
  gtest
  gtest_main
)

# Build Tests
add_executable(fly_stereo_benchmark
  benchmark/opt_flow_benchmark.cc
)

target_link_libraries(fly_stereo_benchmark
  fly_stereo_lib
  benchmark::benchmark
)
