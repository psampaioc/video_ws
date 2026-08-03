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

    // <-- FUNÇÃO DE BATCH ATUALIZADA
    std::vector<std::string> recognizeBatch(const std::vector<cv::Mat>& rois);

private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_;

    std::vector<std::string> dictionary_;

    std::vector<const char*> input_node_names_;
    std::vector<const char*> output_node_names_;

    void loadDictionary(const std::string& dict_path);
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
    cv::Rect head_roi_;
    cv::Rect height_roi_;
    cv::Rect speed_roi_;

    // Hardware Capture
    cv::VideoCapture cap_;
    std::thread capture_thread_;
    std::atomic<bool> running_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lon_pub_;

    // Novos Publishers para os recortes
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr head_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr height_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr speed_pub_;

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
