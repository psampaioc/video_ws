#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include <onnxruntime_cxx_api.h>

class TelemetryOcr {
public:
    TelemetryOcr(const std::string& model_path, const std::string& dict_path);
    std::string recognize(const cv::Mat& roi);

private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_;

    std::vector<std::string> dictionary_;

    std::vector<const char*> input_node_names_;
    std::vector<const char*> output_node_names_;
    std::vector<int64_t> input_node_dims_;

    void loadDictionary(const std::string& dict_path);
    std::vector<float> preprocess(const cv::Mat& img, int& target_width);
    std::string postprocess(const float* out_data, const std::vector<int64_t>& out_shape);
};

class UnifiedVisionNode : public rclcpp::Node {
public:
    UnifiedVisionNode();
    ~UnifiedVisionNode();

private:
    // Configurações
    std::string input_mode_;
    std::string device_path_;

    // ROIs Dinâmicos
    cv::Rect rgb_roi_;
    cv::Rect lat_roi_;
    cv::Rect lon_roi_;

    // Hardware Capture
    cv::VideoCapture cap_;
    std::thread capture_thread_;
    std::atomic<bool> running_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lon_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ts_pub_; // Troubleshooting

    // Subscriber (Para datasets .mcap)
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    std::shared_ptr<TelemetryOcr> ocr_;

    // Callbacks e Processamento
    void loadParameters();
    void hardwareLoop();
    void topicCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void processFrame(const cv::Mat& frame, rclcpp::Time stamp);

    // Utilitários
    bool isRectSafe(const cv::Rect& rect, const cv::Mat& frame, const std::string& roi_name);
    void publishImage(const cv::Mat& img, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, rclcpp::Time stamp, const std::string& encoding);
    void logTroubleshooting(const std::string& msg);
};
