#include <random>

#include "benchmark/benchmark.h"
#include "flyStereo/image_processing/opt_flow_cv_gpu.h"
#include "flyStereo/image_processing/opt_flow_vpi_gpu.h"
#include "flyStereo/test/generators.h"
#include "flyStereo/umat.h"

constexpr int num_pts = 500;
const cv::Size imsize(1280, 720);
constexpr int window_size = 21;
constexpr int max_pyramid_level = 3;
constexpr int max_iters = 30;
constexpr bool use_initial_flow = true;

template <typename T>
static void OptFlowTest(benchmark::State& state) {
  auto [prev_frame, curr_frame, prev_pts, curr_pts] = generate_image_points(imsize, num_pts);
  UMat<uint8_t> status(cv::Size(num_pts, 1));

  // Initialize the optical flow object
  T optflow(window_size, max_pyramid_level, max_iters, use_initial_flow);

  optflow.init(prev_frame.d_frame().size());
  // Perform setup here
  for (auto _ : state) {
    // Perform tracking here
    // std::swap(prev_frame, curr_frame);
    optflow.calc(prev_frame, curr_frame, prev_pts, curr_pts, status);
  }
}

// Register the function as a benchmark
BENCHMARK_TEMPLATE1(OptFlowTest, OptFlowVpiGpu);
BENCHMARK_TEMPLATE1(OptFlowTest, OptFlowCvGpu);
// BENCHMARK(OptFlowTest<OptFlowCvGpu>);
// Run the benchmark
BENCHMARK_MAIN();
