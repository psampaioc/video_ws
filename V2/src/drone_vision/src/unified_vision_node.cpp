#include "drone_vision/unified_vision_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <fstream>
#include <numeric>
#include <algorithm>

using json = nlohmann::json;

// ROIs atualizados com as medições exatas
cv::Rect GLOBAL_RGB_ROI(81, 61, 478, 361);
cv::Rect GLOBAL_LAT_ROI(280, 468, 61, 11);
cv::Rect GLOBAL_LON_ROI(345, 467, 54, 11);

// --- Implementação: TelemetryOcr ---

TelemetryOcr::TelemetryOcr(const std::string& model_path, const std::string& dict_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "TelemetryOcr"),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU))
{
    loadDictionary(dict_path);

    // Ativar TensorRT e Fallback para CUDA (Desativado temporariamente para isolar teste em CPU)
    // OrtTensorRTProviderOptions trt_options{};
    // trt_options.device_id = 0;
    // trt_options.trt_max_workspace_size = 2147483648; // 2GB
    // trt_options.trt_fp16_enable = 1; // Precisão FP16 esmaga latência

    session_options_.SetIntraOpNumThreads(1);
    // session_options_.AppendExecutionProvider_TensorRT(trt_options);

    // Iniciar sessão carregando o modelo
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

    // O PP-OCR usa dinamicamente ["x"] para entrada e retorna softmax
    input_node_names_ = {"x"};
    output_node_names_ = {"softmax_2.tmp_0"};

    // Dimensão padrão PP-OCR Rec: [Batch, Channels, Height, Width]
    // A altura é sempre 48. A largura é dinâmica.
    input_node_dims_ = {1, 3, 48, -1};
}

void TelemetryOcr::loadDictionary(const std::string& dict_path) {
    dictionary_.push_back("blank"); // O índice 0 no CTC é reservado para 'blank'
    std::ifstream file(dict_path);
    std::string line;
    while (std::getline(file, line)) {
        dictionary_.push_back(line);
    }
    dictionary_.push_back(" "); // Espaço no final
}

std::vector<float> TelemetryOcr::preprocess(const cv::Mat& img, int& target_width) {
    cv::Mat rgb, resized;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);

    // PP-OCR exige altura = 48. Escalamos a largura proporcionalmente.
    float ratio = (float)rgb.cols / (float)rgb.rows;
    target_width = static_cast<int>(48 * ratio);

    // Larguras têm de ser múltiplos de 32 para a rede processar bem
    target_width = std::max(32, int(std::ceil(target_width / 32.0f) * 32));
    cv::resize(rgb, resized, cv::Size(target_width, 48));

    // Converter para Float e Normalizar ( (pixel - 127.5)/127.5 )
    resized.convertTo(resized, CV_32FC3, 1.0 / 127.5, -1.0);

    // Converter de HWC (OpenCV padrão) para CHW (Formato Tensores PyTorch/ONNX)
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

    input_node_dims_[3] = target_width; // Atualizar eixo dinâmico

    // Criar o tensor a partir do vector em memória
    auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, input_tensor_values.data(), input_tensor_values.size(),
        input_node_dims_.data(), input_node_dims_.size());

    // Fazer a Inferência via GPU/TensorRT (Bloqueante mas demora < 5ms)
    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_node_names_.data(), &input_tensor, 1,
        output_node_names_.data(), 1);

    // Extrair os resultados
    float* out_data = output_tensors[0].GetTensorMutableData<float>();
    auto out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

    return postprocess(out_data, out_shape);
}

std::string TelemetryOcr::postprocess(const float* out_data, const std::vector<int64_t>& out_shape) {
    // out_shape é [batch_size (1), sequencia_tempo (W), num_classes (len(dict))]
    int64_t seq_len = out_shape[1];
    int64_t num_classes = out_shape[2];

    std::string result = "";
    int last_index = 0;

    // Descodificação CTC Greedy (Procura a maior probabilidade por timestep)
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

        // Regras CTC: Ignorar o caracter 'blank' (idx 0) e ignorar duplicados consecutivos
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

UnifiedVisionNode::UnifiedVisionNode() : Node("unified_vision_node"), state_(RECONNECTING) {
    this->declare_parameter<std::string>("device_path", "/dev/video4");
    this->declare_parameter<double>("fps", 60.0);

    device_path_ = this->get_parameter("device_path").as_string();
    double fps = this->get_parameter("fps").as_double();

    raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb_roi", 10);
    telemetry_pub_ = this->create_publisher<std_msgs::msg::String>("/telemetry/data", 10);

    // Instanciar o OCR com os paths absolutos que definimos no passo 2
    try {
        ocr_ = std::make_shared<TelemetryOcr>("/opt/ocr_models/en_PP-OCRv4_rec.onnx",
                                              "/opt/ocr_models/en_dict.txt");
        RCLCPP_INFO(this->get_logger(), "ONNX CPU OCR Inicializado com Sucesso.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Falha ao carregar OCR: %s", e.what());
    }

    int timer_ms = static_cast<int>(1000.0 / fps);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&UnifiedVisionNode::captureLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Nó Unificado Iniciado (60 FPS Target). Alvo: %s", device_path_.c_str());
}

void UnifiedVisionNode::captureLoop() {
    if (state_ == RECONNECTING) {
        if (!cap_.isOpened()) {
            if (device_path_.find_first_not_of("0123456789") == std::string::npos) {
                cap_.open(std::stoi(device_path_));
            } else {
                cap_.open(device_path_);
            }
        }
        if (!cap_.isOpened()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "A aguardar sinal de vídeo em %s...", device_path_.c_str());
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Sinal de vídeo restabelecido!");
            state_ = CONNECTED;
        }
    }

    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Sinal de vídeo perdido. A libertar hardware.");
        cap_.release();
        state_ = RECONNECTING;
        return;
    }

    auto stamp = this->now();

    publishImage(frame, raw_pub_, stamp, "bgr8");

    if (isRectSafe(GLOBAL_RGB_ROI, frame)) {
        cv::Mat rgb_roi = frame(GLOBAL_RGB_ROI);
        publishImage(rgb_roi, rgb_pub_, stamp, "bgr8");
    }

    std::string lat_result = "N/A";
    std::string lon_result = "N/A";

    // Só corre a inferência se o OCR tiver inicializado corretamente
    if (ocr_ && isRectSafe(GLOBAL_LAT_ROI, frame) && isRectSafe(GLOBAL_LON_ROI, frame)) {
        try {
            lat_result = ocr_->recognize(frame(GLOBAL_LAT_ROI));
            lon_result = ocr_->recognize(frame(GLOBAL_LON_ROI));
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Falha OCR: %s", e.what());
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

bool UnifiedVisionNode::isRectSafe(const cv::Rect& rect, const cv::Mat& frame) {
    return (rect.x >= 0 && rect.y >= 0 &&
            rect.x + rect.width <= frame.cols &&
            rect.y + rect.height <= frame.rows);
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
