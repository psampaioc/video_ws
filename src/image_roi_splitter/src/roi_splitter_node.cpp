#include "image_roi_splitter/roi_splitter_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

RoiSplitterNode::RoiSplitterNode() : Node("roi_splitter_node")
{
  sub_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10,
    std::bind(&RoiSplitterNode::image_callback, this, std::placeholders::_1));

  pub_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb_roi", 10);
  pub_loc_ = this->create_publisher<sensor_msgs::msg::Image>("/telemetry/location_roi", 10);

  RCLCPP_INFO(this->get_logger(), "ROI Splitter Node iniciado. ROIs calibrados (1920x1080).");
}

void RoiSplitterNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat raw_img = cv_ptr->image;

  // ROIs calibrados a partir de resolution_image_live.png (1920x1080)
  cv::Rect rgb_rect(240, 69, 1439, 876);        // Tela principal RGB
  cv::Rect lat_rect(840, 1048, 185, 28);        // Texto latitude
  cv::Rect lon_rect(1032, 1047, 163, 29);       // Texto longitude

  if (raw_img.cols <= rgb_rect.x + rgb_rect.width || raw_img.rows <= rgb_rect.y + rgb_rect.height ||
      raw_img.cols <= lat_rect.x + lat_rect.width || raw_img.rows <= lat_rect.y + lat_rect.height ||
      raw_img.cols <= lon_rect.x + lon_rect.width || raw_img.rows <= lon_rect.y + lon_rect.height) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Dimensoes da imagem incompativeis com as ROIs. Aguardando calibracao.");
    return;
  }

  cv::Mat crop_rgb = raw_img(rgb_rect);
  cv::Mat crop_lat = raw_img(lat_rect);
  cv::Mat crop_lon = raw_img(lon_rect);

  // Redimensiona longitude para largura da latitude antes de empilhar (vconcat exige mesma largura)
  if (crop_lat.cols != crop_lon.cols) {
    cv::resize(crop_lon, crop_lon, cv::Size(crop_lat.cols, crop_lon.rows));
  }

  cv::Mat crop_loc;
  cv::vconcat(crop_lat, crop_lon, crop_loc);

  std_msgs::msg::Header original_header = msg->header;

  auto msg_rgb = cv_bridge::CvImage(original_header, "bgr8", crop_rgb).toImageMsg();
  auto msg_loc = cv_bridge::CvImage(original_header, "bgr8", crop_loc).toImageMsg();

  pub_rgb_->publish(*msg_rgb);
  pub_loc_->publish(*msg_loc);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoiSplitterNode>());
  rclcpp::shutdown();
  return 0;
}
