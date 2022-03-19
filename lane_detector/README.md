# 车道线识别（OpenCV 实现）

## Dependencies

- [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/galactic/) (middleware for robotics)

## Building

	rosdep install --from-paths src --ignore-src -r -y
	colcon build --symlink-install --packages-select lane_detector

## Run with your own video

```bash
ros2 launch lane_detector detector.launch.py video_path:=${YOUR_VIDEO_PATH}
```

## Config files

~/config/default.yaml:

- lower_boundary: HLS的最小值

- upper_boundary: HLS的最大值

- max_contour_area: 车道线轮廓的最大面积

- max_contour_distance: 车道线轮廓之间的最大距离

## 查看输出视频

使用 rqt_image_view 订阅 `/lane_detection` 话题

    ros2 run rqt_image_view rqt_image_view

## 效果

https://www.bilibili.com/video/BV1QF411s7LK/

## 未完成事项

1. 将提取出的车道线轮廓合并，计算出中点坐标
2. 增加 subscription 以订阅 image msg 的形式工作
3. 订阅 camera info，用 solvePnP 的方法完成车道线的三维重建
