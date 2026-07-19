#ifndef HDMI_CAMERA_DRIVER__CAMERA_PUBLISHER_NODE_HPP_
#define HDMI_CAMERA_DRIVER__CAMERA_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/videoio.hpp>

class CameraPublisherNode : public rclcpp::Node
{
public:
  CameraPublisherNode();

private:
  void timer_callback();

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int width;
  int height;
  double fps;
};

#endif  // HDMI_CAMERA_DRIVER__CAMERA_PUBLISHER_NODE_HPP_
