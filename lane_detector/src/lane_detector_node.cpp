// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <thread>

#include "lane_detector/lane_detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace lane_detector
{
class LaneDetectorNode : public rclcpp::Node
{
public:
  LaneDetectorNode(const rclcpp::NodeOptions & options) : Node("lane_detector_node", options)
  {
    RCLCPP_INFO(get_logger(), "Starting LaneDetectorNode!");

    lane_detector_ = std::make_shared<LaneDetector>();

    auto video_path = this->declare_parameter<std::string>("video_path");

    result_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane_detection", 10);

    detect_thread_ = std::thread([this, video_path]() {
      RCLCPP_INFO(get_logger(), "Video path: %s", video_path.c_str());
      cv::VideoCapture capture(video_path);
      if (!capture.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open video file!");
        return;
      }

      cv::Mat frame;
      while (rclcpp::ok()) {
        capture >> frame;
        if (frame.empty()) {
          RCLCPP_ERROR(get_logger(), "Failed to read frame!");
          return;
        }

        auto start_time = this->now();

        auto bin_image = lane_detector_->preprocess(frame);
        auto contours = lane_detector_->getArrow(bin_image);

        auto end_time = this->now();
        auto latency = (end_time - start_time).seconds() * 1e3;
        cv::putText(
          frame, std::to_string(latency) + "ms", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.5,
          cv::Scalar(0, 0, 255), 2);
        cv::drawContours(
          frame, contours, -1, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8, cv::noArray(), 100,
          cv::Point(0, frame.rows / 2));

        result_image_pub_->publish(
          *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());
      }
    });
  }

  ~LaneDetectorNode()
  {
    if (detect_thread_.joinable()) {
      detect_thread_.join();
    }
  }

private:
  std::shared_ptr<LaneDetector> lane_detector_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub_;

  std::thread detect_thread_;
};
}  // namespace lane_detector

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(lane_detector::LaneDetectorNode)