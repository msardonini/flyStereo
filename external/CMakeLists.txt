
# Thread pool does not have support for cmake, add it here
add_library(thread_pool INTERFACE)
target_include_directories(thread_pool
  INTERFACE
    thread-pool
)

if (COVERAGE)
  include(cmake-gtest-coverage-example/cmake/modules/CodeCoverage.cmake)
endif()
