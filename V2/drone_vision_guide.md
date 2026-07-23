# nified Vision Node (V2)

## 1. Visão Geral do Sistema

O `unified_vision_node` é o módulo de perceção tática. Ingere vídeo bruto (hardware ou dataset `.mcap`), isola as regiões de interesse (ROIs) que contêm Latitude e Longitude, e corre uma rede neuronal (OCR) estritamente em CPU para extrair coordenadas. Opera num modelo "Event-Driven" (acionado por frame), sem loops de *polling*.

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
    └── V2/                       # Ambiente ROS 2 ativo.
        ├── build/                # Binários (colcon).
        ├── install/              # Ficheiros de setup (setup.bash).
        └── src/
            └── drone_vision/     # Pacote ROS 2.
                ├── CMakeLists.txt
                ├── package.xml
                ├── include/drone_vision/unified_vision_node.hpp
                └── src/unified_vision_node.cpp

```

## 3. Memória Institucional: OCR v4 vs. v3

* **A Falha do v4:** O modelo PaddleOCR v4 exportado para ONNX com largura dinâmica (`-1`) causou um crash matemático fatal no C++ (`[ShapeInferenceError] All inputs to Concat must have same rank`). Tentativas de contornar com otimizadores (`onnxsim`, `onnx_graphsurgeon`) falharam devido aos bloqueios de ambiente Python (PEP 668) do Ubuntu 24.04.
* **A Solução (v3):** Regressão para o PaddleOCR v3. A sua arquitetura CRNN lida nativamente com tensores dinâmicos no ONNX. A otimização de grafos foi desativada no código (`ORT_DISABLE_ALL`) para garantir estabilidade absoluta.

---

## 4. Referência de Parâmetros (Configuração Dinâmica)

O nó foi construído para ser parametrizável via terminal, sem necessidade de recompilar o C++.

**Argumento Base para injetar parâmetros:** `--ros-args -p <parametro>:=<valor>`

| Parâmetro | Tipo | Valor Padrão | Descrição |
| --- | --- | --- | --- |
| **`input_mode`** | `string` | `"hardware"` | Define a fonte de dados. Usar `"hardware"` para capturar de câmaras via V4L2, ou `"topic"` para ler ficheiros `.mcap` via ROS bag. |
| **`device_path`** | `string` | `"/dev/video4"` | Caminho do dispositivo de vídeo. Relevante apenas se `input_mode` for `"hardware"`. Pode ser um ID (ex: `"0"`) ou um caminho Linux. |
| **`roi_rgb`** | `int array` | `[81, 61, 478, 361]` | Coordenadas de corte central [x, y, largura, altura]. Exclui a UI do ecrã. |
| **`roi_lat`** | `int array` | `[280, 468, 61, 11]` | Coordenadas do corte cirúrgico da Latitude [x, y, w, h]. |
| **`roi_lon`** | `int array` | `[345, 467, 54, 11]` | Coordenadas do corte cirúrgico da Longitude [x, y, w, h]. |

**Exemplo de execução com múltiplos parâmetros:**

```bash
ros2 run drone_vision unified_vision_node --ros-args -p input_mode:="topic" -p roi_lon:="[350, 467, 50, 11]"

```

---

## 5. Guia de Teste Profissional (Setup de Monitorização)

Para auditar a *pipeline* de ponta a ponta, deves dividir a tua área de trabalho em 5 terminais distintos.

**Pré-requisito para TODOS os terminais:**

```bash
source /opt/ros/humble/setup.bash
source /workspace/V2/install/setup.bash

```

### Passo 1: Lançar o Nó (Terminal 1)

Inicia o motor de visão no modo subscrição.

```bash
ros2 run drone_vision unified_vision_node --ros-args -p input_mode:="topic"

```

### Passo 2: Monitorizar a Matemática / OCR (Terminal 2)

Verifica as strings extraídas em tempo real.

```bash
ros2 topic echo /telemetry/data

```

### Passo 3: Monitorizar a Segurança (Terminal 3)

Escuta logs de falha (Crashes ONNX, OpenCV ou erros de limites geométricos nas ROIs).

```bash
ros2 topic echo /troubleshooting

```

### Passo 4: Monitorizar a Geometria Visão (Terminal 4)

Lança a interface gráfica para validar os tensores de imagem que entram no OCR.

```bash
rqt

```

*No rqt, vai a `Plugins > Visualization > Image View`. Abre 3 janelas e seleciona os tópicos:*

1. `/camera/rgb_roi` (Visão desobstruída).
2. `/camera/lat_roi` (Foco no número da Latitude).
3. `/camera/lon_roi` (Foco no número da Longitude).

### Passo 5: Injetar os Dados (Terminal 5)

Arranca a simulação do dataset.

```bash
ros2 bag play /caminho/para/o/ficheiro.mcap

```
