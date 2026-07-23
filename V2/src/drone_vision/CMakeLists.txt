cmake_minimum_required(VERSION 3.8)
project(drone_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Encontrar pacotes ROS 2 e dependências externas
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)

# Encontrar OpenCV local
find_package(OpenCV REQUIRED)

# Encontrar a biblioteca do ONNX Runtime instalada no Docker
find_library(ONNXRUNTIME_LIB NAMES onnxruntime PATHS /usr/local/lib REQUIRED)

# Incluir a pasta de headers do pacote e dependências manuais
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include  # Garante que encontra o onnxruntime_cxx_api.h
)

# Criar o executável a partir do .cpp
add_executable(unified_vision_node src/unified_vision_node.cpp)

# Ligar bibliotecas do OpenCV e ONNX Runtime (CRÍTICO)
target_link_libraries(unified_vision_node
  ${OpenCV_LIBRARIES}
  ${ONNXRUNTIME_LIB}
)

# Adicionar as dependências ROS 2 e JSON ao alvo
ament_target_dependencies(unified_vision_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  nlohmann_json
)

# Regras de instalação para o executável
install(TARGETS unified_vision_node
  DESTINATION lib/${PROJECT_NAME}
)

# Regras de instalação para os headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
