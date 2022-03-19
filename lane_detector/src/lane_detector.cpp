// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "lane_detector/lane_detector.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

// STL
#include <algorithm>
#include <vector>

namespace lane_detector
{
cv::Mat LaneDetector::preprocess(const cv::Mat & image)
{
  auto roi = image(cv::Rect(0, image.rows / 2, image.cols, image.rows / 2));

  cv::Mat hls_image;
  cv::cvtColor(roi, hls_image, cv::COLOR_BGR2HLS);

  cv::Mat bin_image;
  cv::inRange(hls_image, lower_boundary, upper_boundary, bin_image);

  auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 5));
  cv::morphologyEx(bin_image, bin_image, cv::MORPH_CLOSE, element);

  return bin_image;
}

std::vector<std::vector<cv::Point> > LaneDetector::getArrow(const cv::Mat & image)
{
  using std::vector;
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Get bottom point
  vector<contour> contours_plus;
  contours_plus.reserve(contours.size());
  for (const auto & contour : contours) {
    contours_plus.emplace_back(contour);
  }

  // Erase too large contours
  contours_plus.erase(
    std::remove_if(
      contours_plus.begin(), contours_plus.end(),
      [&](const contour & contour) {
        return contour.area > max_contour_area || contour.size() < 5 ||
               (contour.bottom.y < image.rows / 2 && contour.area > 1000);
      }),
    contours_plus.end());

  // Sort by bottom point
  std::sort(contours_plus.begin(), contours_plus.end(), [](const contour & a, const contour & b) {
    return a.bottom.y > b.bottom.y;
  });

  contour last_arrow = contours_plus[0];
  if (last_arrow.bottom.y < image.rows / 2) {
    return {};
  }

  vector<contour> arrows{last_arrow};
  for (size_t i = 1; i < contours_plus.size(); ++i) {
    if (cv::norm(contours_plus[i].bottom - last_arrow.bottom) < max_contour_distance) {
      arrows.emplace_back(contours_plus[i]);
      last_arrow = contours_plus[i];
    }
  }

  contours = vector<vector<cv::Point> >(arrows.begin(), arrows.end());
  return contours;
}

}  // namespace lane_detector
