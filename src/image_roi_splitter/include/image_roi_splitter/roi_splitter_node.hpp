#ifndef IMAGE_ROI_SPLITTER__ROI_SPLITTER_NODE_HPP_
#define IMAGE_ROI_SPLITTER__ROI_SPLITTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class RoiSplitterNode : public rclcpp::Node
{
public:
  RoiSplitterNode();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_loc_;
};

#endif  // IMAGE_ROI_SPLITTER__ROI_SPLITTER_NODE_HPP_
