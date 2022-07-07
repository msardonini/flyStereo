#pragma once

#include "flyStereo/sensor_io/log_params.h"
#include "flyStereo/types/umat.h"

template <UMatDerivative ImageT>
class StereoSystemSinkInterface {
 public:
  virtual int ProcessFrame(const LogParams<ImageT> &params) = 0;

 private:
};
