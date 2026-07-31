Aqui tens o `guide.md` atualizado com a árvore de ficheiros corrigida e a nova secção de build incluída.

---

# Unified Vision Node (V2)

## 1. Visão Geral do Sistema

O `unified_vision_node` é o módulo de perceção tática. Ingere vídeo bruto (hardware ou dataset `.mcap`), isola as regiões de interesse (ROIs) da telemetria, e corre uma rede neuronal (OCR) estritamente em CPU para extrair dados críticos. Opera num modelo "Event-Driven" (acionado por frame), sem loops de *polling*.

**Campos extraídos em tempo real:**

* Latitude e Longitude
* Heading (Bússola / Orientação)
* Height (Altitude / Altura)
* Horizontal Speed (H.S.) e Vertical Speed (V.S.)

## 2. Topologia de Ficheiros

O ambiente está isolado em Docker, segmentado em armazenamento estático (`/opt`) e código mutável (`/workspace`):

```text
/
├── opt/
│   └── ocr_models/               # Modelos protegidos de recompilações acidentais.
│       ├── v3_en_rec.onnx        # (ATUAL) Modelo ONNX do PaddleOCR v3.
│       └── en_dict.txt           # Dicionário alfanumérico.
│
└── workspace/
    ├── roi_calibration/
    │   ├── roi_calibration.py    # Script de calibração interativa para as ROIs.
    │   └── roi_calibration.txt   # Ficheiro de output da calibração.
    └── V2/                       # Ambiente ROS 2 ativo.
        ├── build/                # Binários (colcon).
        ├── install/              # Ficheiros de setup (setup.bash).
        ├── config.yaml           # Ficheiro de parâmetros central (ROIs e Settings).
        └── src/
            └── drone_vision/     # Pacote ROS 2.
                ├── CMakeLists.txt
                ├── package.xml
                ├── include/drone_vision/unified_vision_node.hpp
                └── src/unified_vision_node.cpp

```

## 3. Memória Institucional: OCR v4 vs. v3

* **A Falha do v4:** O modelo PaddleOCR v4 exportado para ONNX com largura dinâmica (`-1`) causou um crash matemático fatal no C++ (`[ShapeInferenceError] All inputs to Concat must have same rank`). Tentativas de contornar com otimizadores (`onnxsim`, `onnx_graphsurgeon`) falharam devido aos bloqueios de ambiente Python (PEP 668) do Ubuntu 24.04.
* **A Solução (v3):** Regressão para o PaddleOCR v3. A sua arquitetura CRNN lida nativamente com tensores dinâmicos no ONNX. A otimização de grafos foi desativada no código (`ORT_DISABLE_ALL`) para garantir estabilidade absoluta, e os logs silenciados (`ORT_LOGGING_LEVEL_ERROR`).

---

## 4. Configuração e Calibração Dinâmica (YAML)

O nó foi reestruturado para não exigir recompilação do C++ quando se calibram novas posições no ecrã. Todos os parâmetros vivem agora no ficheiro `/workspace/V2/config.yaml`.

### Formato do `config.yaml`

```yaml
/unified_vision_node:
  ros__parameters:
    input_mode: "topic"
    device_path: "/dev/video4"
    roi_rgb: [81, 61, 478, 361]
    roi_lat: [280, 468, 61, 11]
    roi_lon: [345, 467, 54, 11]
    roi_heading: [0, 0, 50, 20]
    roi_height: [0, 0, 50, 20]
    roi_hs: [0, 0, 50, 20]
    roi_vs: [0, 0, 50, 20]

```

### Script de Calibração (`roi_calibration.py`)

Para recalibrar, usa o script Python localizado em `/workspace/roi_calibration/`. Ele força a imagem para a resolução do hardware (1920x1080) e gera a formatação exata para colar no YAML:

```bash
cd /workspace/roi_calibration/
python3 roi_calibration.py [caminho_da_imagem]

```

---

## 5. Processo de Build (Compilação)

Sempre que houver alterações diretas no código-fonte C++ (`.cpp` ou `.hpp`), o pacote deve ser recompilado para aplicar as mudanças estruturais.

```bash
cd /workspace/V2
colcon build --packages-select drone_vision
source install/setup.bash

```

*Nota: Alterações apenas no ficheiro `config.yaml` não requerem recompilação, basta reiniciar o nó.*

---

## 6. Guia de Teste (Setup de Monitorização)

Para auditar a *pipeline* de ponta a ponta, divide a tua área de trabalho em 5 terminais distintos.

**Pré-requisito para TODOS os terminais:**

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/V2/install/setup.bash
cd /workspace/V2

```

### Passo 1: Lançar o Nó (Terminal 1)

Inicia o motor de visão passando o ficheiro de configuração centralizado.

```bash
ros2 run drone_vision unified_vision_node --ros-args --params-file config.yaml

```

### Passo 2: Monitorizar a Matemática / OCR (Terminal 2)

Verifica o JSON completo com Latitude, Longitude, Heading, Altura e Velocidades.

```bash
ros2 topic echo /telemetry/data

```

### Passo 3: Monitorizar a Segurança (Terminal 3)

Escuta logs de falha (Crashes ONNX, OpenCV ou erros geométricos nas ROIs).

```bash
ros2 topic echo /troubleshooting

```

### Passo 4: Monitorizar a Geometria Visão (Terminal 4)

Lança a interface gráfica para validar os tensores de imagem que entram no OCR.

```bash
rqt

```

*No rqt, vai a `Plugins > Visualization > Image View`. Abre múltiplas janelas e seleciona os tópicos para auditoria visual:*

1. `/camera/rgb_roi` (Visão desobstruída).
2. `/camera/lat_roi` & `/camera/lon_roi` (Coordenadas).
3. `/camera/heading_roi` & `/camera/height_roi` (Bússola e Altitude).
4. `/camera/hs_roi` & `/camera/vs_roi` (Velocidades).

### Passo 5: Injetar os Dados (Terminal 5)

Se o modo for `"topic"`, arranca a simulação do dataset.

```bash
ros2 bag play /caminho/para/o/ficheiro.mcap

```
