#pragma once

#include <vector>

#include "opencv2/core/types.hpp"
#include "flyStereo/interface.h"


void DrawPoints(const std::vector<cv::Point2f> &mypoints, cv::Mat &myimage);

void DrawPoints(const ImagePoints &mypoints, bool is_cam_0, cv::Mat &myimage);
