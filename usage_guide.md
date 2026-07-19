# video_ws — Guia de Uso Completo

## Visão Geral

Workspace ROS 2 Jazzy para streaming de câmera HDMI e extração de telemetria via ROI splitting.

```
Pacotes:
├── hdmi_camera_driver    # Driver V4L2 → /camera/image_raw (sensor_msgs/Image)
└── image_roi_splitter    # Recorta 3 ROIs → /camera/rgb_roi + /telemetry/location_roi
```

Pipeline:
```
/camera/image_raw (1920x1080 @ 60Hz, bgr8)
       │
       ├──► /camera/rgb_roi          (1439x876)  ──► Visão computacional / exibição
       │
       └──► /telemetry/location_roi  (185x57)    ──► OCR (latitude + longitude empilhados)
```

---

## 1. Calibração dos ROIs (Fazer UMA VEZ)

### 1.1 Capturar Frame de Referência
```bash
# Com a câmera rodando, capture um frame:
ros2 topic echo /camera/image_raw --once > frame_raw.txt
# Ou use o image_view para salvar:
ros2 run image_view image_saver image:=/camera/image_raw __name:=saver
# Gera: frame_0001.png (1920x1080)
```

### 1.2 Rodar Script de Calibração Interativo
```bash
# No container (com X11 habilitado):
cd /workspace/video_ws/roi_calibration
python3 calibrate_rois.py ../frame_0001.png
# Ou usa a imagem padrão:
python3 calibrate_rois.py
```

**Como usar:**
1. Janela abre com a imagem (redimensionada para caber na tela)
2. **Clique 2 pontos por ROI** (canto sup.esq. → canto inf.dir.)
3. Ordem: **RGB → Latitude → Longitude**
4. Coordenadas aparecem no terminal
5. Pressione **ESC** para salvar e sair

### 1.3 Resultado
Gera/atualiza: `roi_calibration/roi_calibration.txt`
```
# Calibração ROIs - frame_0001.png
# Resolução: 1920x1080
# Formato: nome x y w h

rgb_roi: 240 69 1439 876
latitude_roi: 840 1048 185 28
longitude_roi: 1032 1047 163 29
```

### 1.4 Atualizar Código (Automático)
Os valores **já estão hardcoded** no `roi_splitter_node.cpp` (linhas 30-32). Se recalibrar:
1. Copie os 3 valores do `.txt`
2. Atualize no `src/image_roi_splitter/src/roi_splitter_node.cpp`:
```cpp
cv::Rect rgb_rect(240, 69, 1439, 876);
cv::Rect lat_rect(840, 1048, 185, 28);
cv::Rect lon_rect(1032, 1047, 163, 29);
```
3. Rebuild: `colcon build --packages-select image_roi_splitter --symlink-install`

> **Futuro:** Tornar configurável via parâmetros ROS 2 (próximo step do roadmap).

---

## 2. Build do Workspace

### Container de Desenvolvimento (Recomendado)
```bash
# HOST - Habilita GUI no container:
xhost +local:root

# HOST - Sobe container persistente:
docker run -d --rm \
  --name ros2_dev \
  --net=host --ipc=host --pid=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/psampaioc/Workspaces:/workspace \
  -w /workspace/video_ws \
  rosstudy_env:jazzy \
  sleep infinity

# HOST - Entra no container:
docker exec -it ros2_dev bash
```

### Build
```bash
# Dentro do container:
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hdmi_camera_driver image_roi_splitter --symlink-install
source install/setup.bash
```

### Build Limpo (Para Testar do Zero)
```bash
rm -rf build install log
colcon build --packages-select hdmi_camera_driver image_roi_splitter --symlink-install
```

---

## 3. Execução Completa

### Terminal 1 — Driver da Câmera (Requer Hardware)
```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source /workspace/video_ws/install/setup.bash

# Device padrão: /dev/video4 (ajuste se necessário)
ros2 run hdmi_camera_driver camera_publisher_node --ros-args -p device_path:=/dev/video4
```

**Parâmetros disponíveis:**
| Parâmetro | Padrão | Descrição |
|-----------|--------|-----------|
| `device_path` | `/dev/video4` | Device V4L2 |
| `width` | `1920` | Largura |
| `height` | `1080` | Altura |
| `fps` | `60.0` | Taxa de frames |

```bash
# Exemplo com parâmetros customizados:
ros2 run hdmi_camera_driver camera_publisher_node \
  --ros-args -p device_path:=/dev/video4 -p width:=1920 -p height:=1080 -p fps:=60.0
```

### Terminal 2 — ROI Splitter
```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source /workspace/video_ws/install/setup.bash
ros2 run image_roi_splitter roi_splitter_node
```

### Terminal 3 — Verificação
```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash

# Lista tópicos
ros2 topic list
# Deve mostrar:
# /camera/image_raw
# /camera/rgb_roi
# /telemetry/location_roi
# /parameter_events
# /rosout

# Inspeciona RGB ROI (deve ser 1439x876)
ros2 topic echo /camera/rgb_roi --once

# Inspeciona Location ROI (deve ser 185x57 = 28+29 empilhados)
ros2 topic echo /telemetry/location_roi --once
```

### Terminal 4 — Visualização (Opcional, Requer Display)
```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source /workspace/video_ws/install/setup.bash

# Tela principal RGB
ros2 run image_view image_view image:=/camera/rgb_roi

# Telemetria empilhada (lat + lon)
ros2 run image_view image_view image:=/telemetry/location_roi
```

---

## 4. Teste em Container Limpo (Validação para Apresentação)

```bash
# HOST - Container temporário só para testar build+run:
docker run -d --rm \
  --name ros2_test_lab \
  --net=host --ipc=host --pid=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/psampaioc/Workspaces:/workspace \
  -w /workspace/video_ws \
  rosstudy_env:jazzy \
  sleep infinity

# HOST - Habilita X11:
xhost +local:root

# HOST - Build dentro do container limpo:
docker exec ros2_test_lab bash -c "
  source /opt/ros/jazzy/setup.bash &&
  colcon build --packages-select hdmi_camera_driver image_roi_splitter --symlink-install
"

# HOST - Testa driver (precisa device):
docker exec -it ros2_test_lab bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /workspace/video_ws/install/setup.bash &&
  timeout 5 ros2 run hdmi_camera_driver camera_publisher_node --ros-args -p device_path:=/dev/video4
"

# HOST - Testa splitter:
docker exec -it ros2_test_lab bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /workspace/video_ws/install/setup.bash &&
  timeout 5 ros2 run image_roi_splitter roi_splitter_node
"

# HOST - Verifica tópicos:
docker exec ros2_test_lab bash -c "
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic list
"

# HOST - Limpa:
docker stop ros2_test_lab
```

---

## 5. Checklist de Validação (Para Apresentação)

| Item | Comando | Sucesso Se |
|------|---------|------------|
| **Build limpo** | `colcon build ...` | 0 erros, 2 pacotes built |
| **Driver sobe** | `ros2 run hdmi_camera_driver ...` | Log "Publicando frame..." a 60Hz |
| **Splitter sobe** | `ros2 run image_roi_splitter ...` | Log "ROIs calibrados (1920x1080)" |
| **Tópicos existem** | `ros2 topic list` | 4 tópicos incluindo rgb_roi + location_roi |
| **RGB ROI tamanho** | `ros2 topic echo /camera/rgb_roi --once` | `width: 1439`, `height: 876` |
| **Location ROI tamanho** | `ros2 topic echo /telemetry/location_roi --once` | `width: 185`, `height: 57` |
| **Timestamps preservados** | `ros2 topic echo /camera/rgb_roi --once` | `header.stamp` == timestamp de captura |
| **Frame ID correto** | `ros2 topic echo /camera/rgb_roi --once` | `header.frame_id: "camera_frame"` |

---

## 6. Próximos Passos (Roadmap)

| Item | Status | Esforço |
|------|--------|---------|
| Launch file compose (driver + splitter) | 📋 Planejado | 30 min |
| Parâmetros dinâmicos (OnSetParametersCallback) | 📋 Planejado | 45 min |
| DiagnosticUpdater (FPS, dropped frames) | 📋 Planejado | 30 min |
| Nó OCR telemetria (ONNX CRNN) | 📋 Planejado | 2-3 hrs |
| Testes unitários (v4l2loopback mock) | 📋 Planejado | 1 hr |
| Launch file pipeline completo | 📋 Planejado | 30 min |

---

## 7. Estrutura do Workspace

```
video_ws/
├── src/
│   ├── hdmi_camera_driver/
│   │   ├── include/hdmi_camera_driver/camera_publisher_node.hpp
│   │   ├── src/camera_publisher_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── image_roi_splitter/
│       ├── include/image_roi_splitter/roi_splitter_node.hpp
│       ├── src/roi_splitter_node.cpp
│       ├── CMakeLists.txt
│       └── package.xml
├── roi_calibration/
│   ├── calibrate_rois.py          # Script interativo
│   ├── roi_calibration.txt        # Valores calibrados (gitignored)
│   └── preview_rois.py            # Preview headless (opcional)
├── build/                          # Colcon build (gitignored)
├── install/                        # Colcon install (gitignored)
├── log/                            # Colcon logs (gitignored)
├── .gitignore
├── CLAUDE.md                       # Instruções do projeto
├── usage_guide.md                  # ESTE ARQUIVO
└── resolution_image_live.png       # Frame de referência (1920x1080)
```

---

## 8. Comandos Úteis Rápidos

```bash
# Ver logs do driver
ros2 topic hz /camera/image_raw

# Ver taxa do splitter
ros2 topic hz /camera/rgb_roi

# Ver parâmetros do driver
ros2 param list /camera_publisher_node
ros2 param get /camera_publisher_node device_path

# Gravar bag
ros2 bag record /camera/image_raw /camera/rgb_roi /telemetry/location_roi

# Reproduzir bag
ros2 bag play arquivo.db3 --loop
```

---

## 9. Troubleshooting

| Problema | Solução |
|----------|---------|
| `cv2.error: Can't initialize GTK backend` | `xhost +local:root` no host antes de `docker exec` |
| Device `/dev/video4` não encontrado | `ls /dev/video*` — ajuste `-p device_path:=...` |
| Imagem preta / sem frames | Verifique cabo HDMI, capture card, `v4l2-ctl --list-devices` |
| ROI fora dos limites | Recalibre com `python3 calibrate_rois.py` — imagem deve ser 1920x1080 |
| Build falha: `cv_bridge` não encontrado | `sudo apt install ros-jazzy-cv-bridge` no Dockerfile base |
| Timestamp não bate | Driver publica `header.stamp = now()` no momento da captura (OK) |

---

*Gerado para apresentação de laboratório — Naval Rex / video_ws*