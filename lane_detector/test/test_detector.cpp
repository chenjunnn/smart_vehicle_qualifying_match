// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "lane_detector/lane_detector.hpp"

lane_detector::LaneDetector detector;
cv::Mat image = cv::imread("/home/chenjun/ros2_ws/test.png");
cv::Mat bin_image;

TEST(test_detector, test_preprocess)
{
  bin_image = detector.preprocess(image);
  cv::imwrite("/tmp/bin.png", bin_image);
}

TEST(test_detector, test_get_arrow)
{
  auto contours = detector.getArrow(bin_image);
  cv::drawContours(
    image, contours, -1, cv::Scalar(0, 0, 255), 2, cv::LINE_8, cv::noArray(), 100,
    cv::Point(0, image.rows / 2));
  cv::imwrite("/tmp/arrow.png", image);
}

TEST(test_detector, test_detect_video)
{
  cv::VideoCapture cap("/home/chenjun/ros2_ws/test.mp4");
  cv::VideoWriter writer;
  writer.open(
    "/tmp/test.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30,
    cv::Size(image.cols, image.rows), true);
  cv::Mat frame;
  while (cap.read(frame)) {
    auto start_time = std::chrono::steady_clock::now();

    bin_image = detector.preprocess(frame);
    auto contours = detector.getArrow(bin_image);

    auto end_time = std::chrono::steady_clock::now();
    auto latency =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1e6;
    cv::putText(
      frame, std::to_string(latency) + "ms", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.5,
      cv::Scalar(0, 0, 255), 2);
    cv::drawContours(
      frame, contours, -1, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8, cv::noArray(), 100,
      cv::Point(0, frame.rows / 2));
    cv::imshow("video", frame);
    cv::waitKey(1);
    writer << frame;
  }
  cap.release();
  writer.release();
}
