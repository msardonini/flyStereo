
#include "flyStereo/timed_logger.hpp"

int main() {
  auto& logger = TimedLogger::get_instance();

  logger.timed_log(1, "hello");
  logger.timed_log(1, "hello");
  logger.timed_log(2, "hello");
  logger.timed_log(3, "hello");
  logger.timed_log(1, "hello");
  logger.timed_log(3, "hello");


  return 0;
}