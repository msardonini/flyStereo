#pragma once

#include <memory>

#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"

struct debug_video_recorder {
  std::unique_ptr<cv::VideoWriter> writer;
  cv::Mat frame_local0;
  cv::Mat frame_local1;

  void DrawPts(cv::Mat frame, int frame_num, std::vector<cv::Point2f> blue_pts, std::vector<cv::Point2f> green_pts = {},
               std::vector<cv::Point2f> red_pts = {}, std::vector<cv::Point2f> cyan_pts = {}) {
    cv::Mat frame_local;
    if (frame_num == 1) {
      frame_local1 = frame.clone();
      cv::cvtColor(frame_local1, frame_local1, cv::COLOR_GRAY2BGR);
      frame_local = frame_local1;
    } else {
      frame_local0 = frame.clone();
      cv::cvtColor(frame_local0, frame_local0, cv::COLOR_GRAY2BGR);
      frame_local = frame_local0;
    }

    int myradius = 5;
    // Draw blue points
    for (size_t i = 0; i < blue_pts.size(); i++) {
      cv::Scalar color = CV_RGB(0, 0, 255);
      circle(frame_local, cv::Point(blue_pts[i].x, blue_pts[i].y), myradius, color, -1, 8, 0);
    }

    // Draw green points
    for (size_t i = 0; i < green_pts.size(); i++) {
      cv::Scalar color = CV_RGB(0, 255, 0);
      circle(frame_local, cv::Point(green_pts[i].x, green_pts[i].y), myradius, color, -1, 8, 0);
    }

    // Draw red points
    for (size_t i = 0; i < red_pts.size(); i++) {
      cv::Scalar color = CV_RGB(255, 0, 0);
      circle(frame_local, cv::Point(red_pts[i].x, red_pts[i].y), myradius, color, -1, 8, 0);
    }

    // Draw cyan points
    for (size_t i = 0; i < cyan_pts.size(); i++) {
      cv::Scalar color = CV_RGB(0, 255, 255);
      circle(frame_local, cv::Point(cyan_pts[i].x, cyan_pts[i].y), myradius, color, -1, 8, 0);
    }
  }

  void WriteFrame() {
    cv::Mat concat_frame;
    cv::hconcat(frame_local0, frame_local1, concat_frame);

    // Draw a vertical line to show the separation between the two cameras
    cv::line(concat_frame, cv::Point(concat_frame.cols / 2, 0), cv::Point(concat_frame.cols / 2, concat_frame.rows),
             cv::Scalar(0, 255, 0), 1, 8, 0);

    // std::cout << "Size " << frame_local0.size() << std::endl;
    // std::cout << "Size " << frame_local1.size() << std::endl;
    // std::cout << "Size " << concat_frame.size() << std::endl;

    if (!writer) {
      writer = std::make_unique<cv::VideoWriter>("./debug_video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 2,
                                                 concat_frame.size(), true);
    }
    if (writer->isOpened()) {
      writer->write(concat_frame);
    } else {
      std::cerr << "failed to open video!!\n";
    }
  }
};