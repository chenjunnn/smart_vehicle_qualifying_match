// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef LANE_DETECTOR__LANE_DETECTOR_HPP_
#define LANE_DETECTOR__LANE_DETECTOR_HPP_

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <vector>

namespace lane_detector
{
class LaneDetector
{
public:
  cv::Mat preprocess(const cv::Mat & image);

  std::vector<std::vector<cv::Point> > getArrow(const cv::Mat & image);

  cv::Scalar lower_boundary;
  cv::Scalar upper_boundary;

  int max_contour_area;
  double max_contour_distance;

private:
  struct contour : std::vector<cv::Point>
  {
    explicit contour(const std::vector<cv::Point> & contour) : std::vector<cv::Point>(contour)
    {
      bottom = contour[0];
      for (const auto & p : contour) {
        bottom = (p.y > bottom.y) ? p : bottom;
      }
      area = cv::contourArea(contour);
    }
    cv::Point bottom;
    double area;
  };
};

}  // namespace lane_detector

#endif  // LANE_DETECTOR__LANE_DETECTOR_HPP_
