#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// Headers da API C++ do ONNX Runtime
#include <onnxruntime_cxx_api.h>

class TelemetryOcr {
public:
    // O construtor vai carregar o modelo e o dicionário
    TelemetryOcr(const std::string& model_path, const std::string& dict_path);
    std::string recognize(const cv::Mat& roi);

private:
    // Estruturas essenciais do ONNXRuntime
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_;

    std::vector<std::string> dictionary_;

    // Variáveis de I/O do modelo
    std::vector<const char*> input_node_names_;
    std::vector<const char*> output_node_names_;
    std::vector<int64_t> input_node_dims_;

    // Funções auxiliares
    void loadDictionary(const std::string& dict_path);
    std::vector<float> preprocess(const cv::Mat& img, int& target_width);
    std::string postprocess(const float* out_data, const std::vector<int64_t>& out_shape);
};

class UnifiedVisionNode : public rclcpp::Node {
public:
    UnifiedVisionNode();

private:
    enum State { CONNECTED, RECONNECTING };
    State state_;
    std::string device_path_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;

    std::shared_ptr<TelemetryOcr> ocr_;

    void captureLoop();
    bool isRectSafe(const cv::Rect& rect, const cv::Mat& frame);
    void publishImage(const cv::Mat& img, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, rclcpp::Time stamp, const std::string& encoding);
};
