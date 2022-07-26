#pragma once

#include "flyStereo/sensor_io/log_params.h"

class StereoSystemSinkInterface {
 public:
  virtual int ProcessFrame(const LogParams &params) = 0;

 private:
};
