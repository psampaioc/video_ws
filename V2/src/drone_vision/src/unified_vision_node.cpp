#include "drone_vision/unified_vision_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <fstream>
#include <numeric>
#include <algorithm>

using json = nlohmann::json;

// --- Implementação: TelemetryOcr ---
TelemetryOcr::TelemetryOcr(const std::string& model_path, const std::string& dict_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "TelemetryOcr"),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU))
{
    loadDictionary(dict_path);

    // Forçar apenas 1 thread por inferência
    session_options_.SetIntraOpNumThreads(1);

    // Desativar a otimização de grafos do ONNX para evitar conflitos de dimensões dinâmicas
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);

    // Iniciar sessão
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

    input_node_names_ = {"x"};
    output_node_names_ = {"fetch_name_0"};
    input_node_dims_ = {1, 3, 48, -1};
}

void TelemetryOcr::loadDictionary(const std::string& dict_path) {
    dictionary_.push_back("blank");
    std::ifstream file(dict_path);
    std::string line;
    while (std::getline(file, line)) {
        dictionary_.push_back(line);
    }
    dictionary_.push_back(" ");
}

std::vector<float> TelemetryOcr::preprocess(const cv::Mat& img, int& target_width) {
    cv::Mat rgb, resized;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);

    float ratio = (float)rgb.cols / (float)rgb.rows;
    target_width = static_cast<int>(48 * ratio);
    target_width = std::max(32, int(std::ceil(target_width / 32.0f) * 32));
    cv::resize(rgb, resized, cv::Size(target_width, 48));

    resized.convertTo(resized, CV_32FC3, 1.0 / 127.5, -1.0);

    std::vector<cv::Mat> channels(3);
    cv::split(resized, channels);

    std::vector<float> input_tensor_values;
    input_tensor_values.reserve(3 * 48 * target_width);
    for (int c = 0; c < 3; ++c) {
        input_tensor_values.insert(input_tensor_values.end(),
                                   (float*)channels[c].datastart,
                                   (float*)channels[c].dataend);
    }
    return input_tensor_values;
}

std::string TelemetryOcr::recognize(const cv::Mat& roi) {
    if (roi.empty()) return "N/A";

    int target_width;
    std::vector<float> input_tensor_values = preprocess(roi, target_width);
    input_node_dims_[3] = target_width;

    auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, input_tensor_values.data(), input_tensor_values.size(),
        input_node_dims_.data(), input_node_dims_.size());

    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_node_names_.data(), &input_tensor, 1,
        output_node_names_.data(), 1);

    float* out_data = output_tensors[0].GetTensorMutableData<float>();
    auto out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

    return postprocess(out_data, out_shape);
}

std::string TelemetryOcr::postprocess(const float* out_data, const std::vector<int64_t>& out_shape) {
    int64_t seq_len = out_shape[1];
    int64_t num_classes = out_shape[2];

    std::string result = "";
    int last_index = 0;

    for (int64_t i = 0; i < seq_len; ++i) {
        int best_idx = 0;
        float max_prob = 0.0f;
        for (int64_t j = 0; j < num_classes; ++j) {
            float prob = out_data[i * num_classes + j];
            if (prob > max_prob) {
                max_prob = prob;
                best_idx = j;
            }
        }

        if (best_idx != 0 && best_idx != last_index) {
            if (best_idx < (int)dictionary_.size()) {
                result += dictionary_[best_idx];
            }
        }
        last_index = best_idx;
    }
    return result;
}

// --- Implementação: UnifiedVisionNode ---
UnifiedVisionNode::UnifiedVisionNode() : Node("unified_vision_node"), running_(true) {
    loadParameters();

    raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb_roi", 10);
    lat_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/lat_roi", 10);
    lon_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/lon_roi", 10);
    telemetry_pub_ = this->create_publisher<std_msgs::msg::String>("/telemetry/data", 10);
    ts_pub_ = this->create_publisher<std_msgs::msg::String>("/troubleshooting", 10);

    try {
        ocr_ = std::make_shared<TelemetryOcr>("/opt/ocr_models/v3_en_rec.onnx", "/opt/ocr_models/en_dict.txt");
        RCLCPP_INFO(this->get_logger(), "ONNX CPU OCR Inicializado com Sucesso (v3).");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Falha ao carregar OCR: %s", e.what());
    }

    if (input_mode_ == "hardware") {
        RCLCPP_INFO(this->get_logger(), "Modo Event-Driven (Hardware). Aguardando vídeo em: %s", device_path_.c_str());
        capture_thread_ = std::thread(&UnifiedVisionNode::hardwareLoop, this);
    } else if (input_mode_ == "topic") {
        RCLCPP_INFO(this->get_logger(), "Modo Event-Driven (Tópico). Aguardando playback de mcap em /camera/image_raw");
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&UnifiedVisionNode::topicCallback, this, std::placeholders::_1));
    } else {
        RCLCPP_ERROR(this->get_logger(), "input_mode inválido. Escolha 'hardware' ou 'topic'.");
    }
}

UnifiedVisionNode::~UnifiedVisionNode() {
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (cap_.isOpened()) cap_.release();
}

void UnifiedVisionNode::loadParameters() {
    this->declare_parameter<std::string>("input_mode", "hardware");
    this->declare_parameter<std::string>("device_path", "/dev/video4");

    this->declare_parameter<std::vector<int64_t>>("roi_rgb", {81, 61, 478, 361});
    this->declare_parameter<std::vector<int64_t>>("roi_lat", {280, 468, 61, 11});
    this->declare_parameter<std::vector<int64_t>>("roi_lon", {345, 467, 54, 11});

    input_mode_ = this->get_parameter("input_mode").as_string();
    device_path_ = this->get_parameter("device_path").as_string();

    auto r_rgb = this->get_parameter("roi_rgb").as_integer_array();
    auto r_lat = this->get_parameter("roi_lat").as_integer_array();
    auto r_lon = this->get_parameter("roi_lon").as_integer_array();

    rgb_roi_ = cv::Rect(r_rgb[0], r_rgb[1], r_rgb[2], r_rgb[3]);
    lat_roi_ = cv::Rect(r_lat[0], r_lat[1], r_lat[2], r_lat[3]);
    lon_roi_ = cv::Rect(r_lon[0], r_lon[1], r_lon[2], r_lon[3]);
}

void UnifiedVisionNode::logTroubleshooting(const std::string& msg) {
    auto ts_msg = std_msgs::msg::String();
    ts_msg.data = "[TROUBLESHOOTING] " + msg;
    ts_pub_->publish(ts_msg);
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
}

void UnifiedVisionNode::hardwareLoop() {
    while (rclcpp::ok() && running_) {
        if (!cap_.isOpened()) {
            if (device_path_.find_first_not_of("0123456789") == std::string::npos) {
                cap_.open(std::stoi(device_path_));
            } else {
                cap_.open(device_path_);
            }

            if (cap_.isOpened()) {
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
                cap_.set(cv::CAP_PROP_FPS, 60);
            }

            if (!cap_.isOpened()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }

        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            logTroubleshooting("Sinal de vídeo vazio. Hardware falhou.");
            cap_.release();
            continue;
        }

        auto stamp = this->now();
        publishImage(frame, raw_pub_, stamp, "bgr8");
        processFrame(frame, stamp);
    }
}

void UnifiedVisionNode::topicCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        processFrame(frame, msg->header.stamp);
    } catch (cv_bridge::Exception& e) {
        logTroubleshooting("Erro no cv_bridge: " + std::string(e.what()));
    }
}

void UnifiedVisionNode::processFrame(const cv::Mat& frame, rclcpp::Time stamp) {
    if (isRectSafe(rgb_roi_, frame, "RGB_ROI")) {
        publishImage(frame(rgb_roi_), rgb_pub_, stamp, "bgr8");
    }

    std::string lat_result = "N/A";
    std::string lon_result = "N/A";

    bool lat_safe = isRectSafe(lat_roi_, frame, "LAT_ROI");
    bool lon_safe = isRectSafe(lon_roi_, frame, "LON_ROI");

    if (lat_safe && lon_safe) {
        publishImage(frame(lat_roi_), lat_pub_, stamp, "bgr8");
        publishImage(frame(lon_roi_), lon_pub_, stamp, "bgr8");

        if (ocr_) {
            try {
                lat_result = ocr_->recognize(frame(lat_roi_));
                lon_result = ocr_->recognize(frame(lon_roi_));
            } catch (const std::exception& e) {
                logTroubleshooting("Crash na Inferência OCR: " + std::string(e.what()));
            }
        }
    }

    json telemetry_json;
    telemetry_json["timestamp"] = stamp.seconds();
    telemetry_json["telemetry"]["latitude"] = lat_result;
    telemetry_json["telemetry"]["longitude"] = lon_result;

    auto msg_tel = std_msgs::msg::String();
    msg_tel.data = telemetry_json.dump();
    telemetry_pub_->publish(msg_tel);
}

bool UnifiedVisionNode::isRectSafe(const cv::Rect& rect, const cv::Mat& frame, const std::string& roi_name) {
    bool safe = (rect.x >= 0 && rect.y >= 0 &&
                 rect.x + rect.width <= frame.cols &&
                 rect.y + rect.height <= frame.rows);

    if (!safe) {
        std::string err = "Falha Geométrica na ROI " + roi_name +
                          ". Imagem(" + std::to_string(frame.cols) + "x" + std::to_string(frame.rows) +
                          "), mas a ROI pede (" + std::to_string(rect.x + rect.width) + "x" + std::to_string(rect.y + rect.height) + ").";
        logTroubleshooting(err);
    }
    return safe;
}

void UnifiedVisionNode::publishImage(const cv::Mat& img, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, rclcpp::Time stamp, const std::string& encoding) {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = "camera_frame";

    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
    pub->publish(*msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnifiedVisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
