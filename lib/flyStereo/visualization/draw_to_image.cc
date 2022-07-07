#include "flyStereo/visualization/draw_to_image.h"

#include "opencv2/imgproc.hpp"

void DrawPoints(const cv::Mat_<cv::Vec2f> &mypoints, cv::Mat &myimage) {
  int myradius = 5;
  for (auto i = 0; i < mypoints.cols; i++) {
    cv::Scalar color;
    if (i == 0) {
      color = CV_RGB(0, 0, 255);
    } else {
      color = CV_RGB(0, 0, 0);
    }
    circle(myimage, cv::Point(mypoints(0, i)[0], mypoints(0, i)[1]), myradius, color, -1, 8, 0);
  }
}

void DrawPoints(const std::vector<cv::Point2f> &mypoints, cv::Mat &myimage) {
  int myradius = 5;
  for (auto i = 0ul; i < mypoints.size(); i++) {
    cv::Scalar color;
    if (i == 0) {
      color = CV_RGB(0, 0, 255);
    } else {
      color = CV_RGB(0, 0, 0);
    }
    circle(myimage, mypoints[i], myradius, color, -1, 8, 0);
  }
}

void DrawPoints(const UMat<cv::Vec2f> &mypoints, cv::Mat &myimage) { DrawPoints(mypoints.frame(), myimage); }
