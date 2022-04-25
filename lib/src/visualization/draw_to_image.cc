#include "flyStereo/visualization/draw_to_image.h"

#include "opencv2/imgproc.hpp"

void DrawPoints(const std::vector<cv::Point2f> &mypoints, cv::Mat &myimage) {
  int myradius = 5;
  for (size_t i = 0; i < mypoints.size(); i++) {
    cv::Scalar color;
    if (i == 0) {
      color = CV_RGB(0, 0, 255);
    } else {
      color = CV_RGB(0, 0, 0);
    }
    circle(myimage, cv::Point(mypoints[i].x, mypoints[i].y), myradius, color, -1, 8, 0);
  }
}
void DrawPoints(const ImagePoints &mypoints, bool is_cam_0, cv::Mat &myimage) {
  std::vector<cv::Point2f> draw_points;
  draw_points.reserve(mypoints.pts.size());
  for (auto &point : mypoints.pts) {
    if (is_cam_0) {
      draw_points.push_back(cv::Point2f(point.cam0_t1[0], point.cam0_t1[1]));
    } else {
      draw_points.push_back(cv::Point2f(point.cam1_t1[0], point.cam1_t1[1]));
    }
  }
  DrawPoints(draw_points, myimage);
}