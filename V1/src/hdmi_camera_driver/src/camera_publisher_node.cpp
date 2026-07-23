#include "hdmi_camera_driver/camera_publisher_node.hpp"
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include <memory>

CameraPublisherNode::CameraPublisherNode() : Node("camera_publisher_node")
{
  this->declare_parameter<std::string>("device_path", "/dev/video4");
  this->declare_parameter<int>("width", 1920);
  this->declare_parameter<int>("height", 1080);
  this->declare_parameter<double>("fps", 60.0);

  std::string device_path = this->get_parameter("device_path").as_string();
  width = this->get_parameter("width").as_int();
  height = this->get_parameter("height").as_int();
  fps = this->get_parameter("fps").as_double();

  int api_preference = cv::CAP_ANY;

      // Se for hardware, força V4L2 para minimizar latência. Se for ficheiro, usa o descodificador automático.
      if (device_path.find("/dev/video") == 0) {
        api_preference = cv::CAP_V4L2;
      }

      cap_.open(device_path, api_preference);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open video device: %s", device_path.c_str());
    return;
  }

  // Configuração para processamento de visão (raw/YUYV), sem compressão MJPEG.
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap_.set(cv::CAP_PROP_FPS, fps);
  cap_.set(cv::CAP_PROP_BUFFERSIZE, 1); // Minimiza latência de hardware

  int actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
  int actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
  double actual_fps = cap_.get(cv::CAP_PROP_FPS);
  RCLCPP_INFO(this->get_logger(), "Opened %s: %dx%d @ %.1f FPS (requested: %dx%d @ %.1f FPS)",
              device_path.c_str(), actual_width, actual_height, actual_fps, width, height, fps);

  pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

  auto period_ms = static_cast<int>(1000.0 / fps);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&CameraPublisherNode::timer_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Camera publisher node started. Pipeline ativa.");
}

void CameraPublisherNode::timer_callback()
{
  cv::Mat frame;
  // Bloqueia até o frame estar fisicamente na memória
  if (!cap_.read(frame) || frame.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to grab frame");
    return;
  }

  // Timestamp capturado no instante zero após leitura, crucial para sincronização
  auto stamp = this->get_clock()->now();

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  msg->header.stamp = stamp;
  msg->header.frame_id = "camera_frame";

  pub_->publish(*msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
