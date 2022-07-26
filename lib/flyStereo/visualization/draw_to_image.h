#pragma once

#include <vector>

#include "flyStereo/interface.h"
#include "flyStereo/types/umat.h"
#include "opencv2/core/types.hpp"

void DrawPoints(const cv::Mat_<cv::Vec2f> &mypoints, cv::Mat &myimage);
void DrawPoints(const std::vector<cv::Point2f> &mypoints, cv::Mat &myimage);

void DrawPoints(const UMat<cv::Vec2f> &mypoints, cv::Mat &myimage);
