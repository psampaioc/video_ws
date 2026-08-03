#include "drone_vision/unified_vision_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>

using json = nlohmann::json;

// --- Implementação: TelemetryOcr ---
TelemetryOcr::TelemetryOcr(const std::string& model_path, const std::string& dict_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "TelemetryOcr"),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU))
{
    loadDictionary(dict_path);

    session_options_.SetIntraOpNumThreads(4);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // MANTEMOS O CUDA PURO PARA COMPARAR ALHOS COM ALHOS
    OrtCUDAProviderOptions cuda_options;
    cuda_options.device_id = 0;
    cuda_options.arena_extend_strategy = 0;
    cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
    cuda_options.do_copy_in_default_stream = 1;

    try {
        session_options_.AppendExecutionProvider_CUDA(cuda_options);
    } catch (const std::exception& e) {
    }

    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

    input_node_names_ = {"x"};
    output_node_names_ = {"fetch_name_0"};
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

std::vector<std::string> TelemetryOcr::recognizeBatch(const std::vector<cv::Mat>& rois) {
    int batch_size = 0;
    std::vector<cv::Mat> valid_rois;
    std::vector<int> original_indices;

    // 1. Filtrar ROIs vazias
    for (size_t i = 0; i < rois.size(); ++i) {
        if (!rois[i].empty()) {
            valid_rois.push_back(rois[i]);
            original_indices.push_back(i);
            batch_size++;
        }
    }

    std::vector<std::string> results(rois.size(), "N/A");
    if (batch_size == 0) return results;

    // 2. Encontrar a largura máxima para o Padding
    int max_width = 0;
    std::vector<cv::Mat> resized_rois;
    resized_rois.reserve(batch_size);

    for (const auto& img : valid_rois) {
        cv::Mat rgb, resized;
        cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
        float ratio = (float)rgb.cols / (float)rgb.rows;
        int target_width = static_cast<int>(48 * ratio);
        target_width = std::max(32, int(std::ceil(target_width / 32.0f) * 32));
        cv::resize(rgb, resized, cv::Size(target_width, 48));
        resized.convertTo(resized, CV_32FC3, 1.0 / 127.5, -1.0); // Fundo a -1.0
        resized_rois.push_back(resized);
        if (target_width > max_width) max_width = target_width;
    }

    // 3. Achatar canais e aplicar Padding
    std::vector<float> input_tensor_values;
    input_tensor_values.reserve(batch_size * 3 * 48 * max_width);

    for (const auto& resized : resized_rois) {
        cv::Mat padded;
        if (resized.cols < max_width) {
            // Padding à direita com valor -1.0
            cv::copyMakeBorder(resized, padded, 0, 0, 0, max_width - resized.cols,
                               cv::BORDER_CONSTANT, cv::Scalar(-1.0, -1.0, -1.0));
        } else {
            padded = resized;
        }

        std::vector<cv::Mat> channels(3);
        cv::split(padded, channels);
        for (int c = 0; c < 3; ++c) {
            input_tensor_values.insert(input_tensor_values.end(),
                                       (float*)channels[c].datastart,
                                       (float*)channels[c].dataend);
        }
    }

    // 4. Construir Tensor e Inferir
    std::vector<int64_t> input_dims = {batch_size, 3, 48, max_width};
    auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, input_tensor_values.data(), input_tensor_values.size(),
        input_dims.data(), input_dims.size());

    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_node_names_.data(), &input_tensor, 1,
        output_node_names_.data(), 1);

    // 5. Pós-processamento Desacoplado [b, seq, classes]
    float* out_data = output_tensors[0].GetTensorMutableData<float>();
    auto out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

    int64_t seq_len = out_shape[1];
    int64_t num_classes = out_shape[2];

    for (int b = 0; b < batch_size; ++b) {
        std::string result = "";
        int last_index = 0;

        for (int64_t i = 0; i < seq_len; ++i) {
            int best_idx = 0;
            float max_prob = 0.0f;

            for (int64_t j = 0; j < num_classes; ++j) {
                float prob = out_data[b * seq_len * num_classes + i * num_classes + j];
                if (prob > max_prob) {
                    max_prob = prob;
                    best_idx = j;
                }
            }
            if (best_idx != 0 && best_idx != last_index && best_idx < (int)dictionary_.size()) {
                result += dictionary_[best_idx];
            }
            last_index = best_idx;
        }
        results[original_indices[b]] = result;
    }

    return results;
}

// --- Implementação: UnifiedVisionNode ---
UnifiedVisionNode::UnifiedVisionNode() : Node("unified_vision_node"), running_(true) {
    loadParameters();

    raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb_roi", 10);
    lat_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/lat_roi", 10);
    lon_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/lon_roi", 10);
    head_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/heading_roi", 10);
    height_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/height_roi", 10);
    speed_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/speed_roi", 10);

    telemetry_pub_ = this->create_publisher<std_msgs::msg::String>("/telemetry/data", 10);
    ts_pub_ = this->create_publisher<std_msgs::msg::String>("/troubleshooting", 10);

    try {
        ocr_ = std::make_shared<TelemetryOcr>("/opt/ocr_models/v3_en_rec.onnx", "/opt/ocr_models/en_dict.txt");
        RCLCPP_INFO(this->get_logger(), "ONNX CUDA OCR Inicializado com Sucesso (Batch Mode).");
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
    this->declare_parameter<std::vector<int64_t>>("roi_heading", {0, 0, 50, 20});
    this->declare_parameter<std::vector<int64_t>>("roi_height", {0, 0, 50, 20});
    this->declare_parameter<std::vector<int64_t>>("roi_speed", {0, 0, 50, 20});

    input_mode_ = this->get_parameter("input_mode").as_string();
    device_path_ = this->get_parameter("device_path").as_string();

    auto r_rgb = this->get_parameter("roi_rgb").as_integer_array();
    auto r_lat = this->get_parameter("roi_lat").as_integer_array();
    auto r_lon = this->get_parameter("roi_lon").as_integer_array();
    auto r_head = this->get_parameter("roi_heading").as_integer_array();
    auto r_height = this->get_parameter("roi_height").as_integer_array();
    auto r_speed = this->get_parameter("roi_speed").as_integer_array();

    rgb_roi_ = cv::Rect(r_rgb[0], r_rgb[1], r_rgb[2], r_rgb[3]);
    lat_roi_ = cv::Rect(r_lat[0], r_lat[1], r_lat[2], r_lat[3]);
    lon_roi_ = cv::Rect(r_lon[0], r_lon[1], r_lon[2], r_lon[3]);
    head_roi_ = cv::Rect(r_head[0], r_head[1], r_head[2], r_head[3]);
    height_roi_ = cv::Rect(r_height[0], r_height[1], r_height[2], r_height[3]);
    speed_roi_ = cv::Rect(r_speed[0], r_speed[1], r_speed[2], r_speed[3]);
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

    std::string lat_result = "N/A", lon_result = "N/A";
    std::string head_result = "N/A", height_result = "N/A", speed_result = "N/A";

    bool lat_safe = isRectSafe(lat_roi_, frame, "LAT_ROI");
    bool lon_safe = isRectSafe(lon_roi_, frame, "LON_ROI");
    bool head_safe = isRectSafe(head_roi_, frame, "HEADING_ROI");
    bool height_safe = isRectSafe(height_roi_, frame, "HEIGHT_ROI");
    bool speed_safe = isRectSafe(speed_roi_, frame, "SPEED_ROI");

    if (lat_safe) publishImage(frame(lat_roi_), lat_pub_, stamp, "bgr8");
    if (lon_safe) publishImage(frame(lon_roi_), lon_pub_, stamp, "bgr8");
    if (head_safe) publishImage(frame(head_roi_), head_pub_, stamp, "bgr8");
    if (height_safe) publishImage(frame(height_roi_), height_pub_, stamp, "bgr8");
    if (speed_safe) publishImage(frame(speed_roi_), speed_pub_, stamp, "bgr8");

    if (ocr_) {
        try {
            std::vector<cv::Mat> rois_to_infer = {
                lat_safe ? frame(lat_roi_) : cv::Mat(),
                lon_safe ? frame(lon_roi_) : cv::Mat(),
                head_safe ? frame(head_roi_) : cv::Mat(),
                height_safe ? frame(height_roi_) : cv::Mat(),
                speed_safe ? frame(speed_roi_) : cv::Mat()
            };

            std::vector<std::string> results = ocr_->recognizeBatch(rois_to_infer);

            lat_result = results[0];
            lon_result = results[1];
            head_result = results[2];
            height_result = results[3];
            speed_result = results[4];

        } catch (const std::exception& e) {
            logTroubleshooting("Crash na Inferência OCR: " + std::string(e.what()));
        }
    }

    json telemetry_json;
    telemetry_json["timestamp"] = stamp.seconds();
    telemetry_json["telemetry"]["latitude"] = lat_result;
    telemetry_json["telemetry"]["longitude"] = lon_result;
    telemetry_json["telemetry"]["heading"] = head_result;
    telemetry_json["telemetry"]["height"] = height_result;
    telemetry_json["telemetry"]["speed"] = speed_result;

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
