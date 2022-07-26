

if (NOT TARGET spdlog::spdlog)
  add_subdirectory(spdlog)
endif()

if (NOT TARGET yaml-cpp)
  add_subdirectory(yaml-cpp)
endif()

# Thread pool does not have support for cmake, add it here
add_library(thread_pool INTERFACE)
target_include_directories(thread_pool
  INTERFACE
    thread-pool
)

if (COVERAGE)
  include(cmake-gtest-coverage-example/cmake/modules/CodeCoverage.cmake)
endif()
