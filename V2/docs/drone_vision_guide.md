# drone_vision Guide (V2)

## Objetivo do pacote

O `drone_vision` processa vídeo, recorta ROIs de telemetria e publica JSON em `/telemetry/data` para o `drone_mapper`.

## Nó do pacote

| Nó | Papel | Essencial? |
|---|---|---|
| `/unified_vision_node` | Captura/consome imagem, OCR, publica telemetria | **Sim** |

## Tópicos do pacote (publicados)

| Tópico | Tipo | Papel | Essencial? | Se remover |
|---|---|---|---|---|
| `/telemetry/data` | `std_msgs/String` | Saída principal para localização/mapa | **Sim** | `drone_mapper` deixa de funcionar |
| `/camera/image_raw` | `sensor_msgs/Image` | Imagem base da pipeline (ou replay) | **Sim** para modo `topic` | Sem frames no pipeline |
| `/camera/rgb_roi` | `sensor_msgs/Image` | ROI visual principal | Opcional | Só perde inspeção visual |
| `/camera/lat_roi` | `sensor_msgs/Image` | ROI latitude | Opcional | Só debug |
| `/camera/lon_roi` | `sensor_msgs/Image` | ROI longitude | Opcional | Só debug |
| `/camera/heading_roi` | `sensor_msgs/Image` | ROI heading | Opcional | Só debug |
| `/camera/height_roi` | `sensor_msgs/Image` | ROI altura | Opcional | Só debug |
| `/camera/speed_roi` | `sensor_msgs/Image` | ROI velocidade | Opcional | Só debug |
| `/troubleshooting` | `std_msgs/String` | Alertas de falha de OCR/geometria | Opcional (recomendado) | Debug fica difícil |

## Tópicos que o pacote consome

| Tópico | Papel | Essencial? |
|---|---|---|
| `/camera/image_raw` | Entrada de frame em `input_mode: topic` | **Sim** neste modo |

## Mínimo do mínimo (KISS)

Para o sistema funcionar com o mínimo:
- **Manter:** `/telemetry/data` (obrigatório), e a entrada de frame conforme modo (`hardware` ou `/camera/image_raw`)
- **Opcional/removível para produção:** todos os tópicos `*_roi` e `/troubleshooting` (se não precisares de debug)

## Comandos

### Build (Docker)
```bash
cd /workspace/V2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drone_vision --symlink-install
source install/setup.bash
```

### Run
```bash
ros2 run drone_vision unified_vision_node --ros-args --params-file /workspace/V2/config.yaml
```

### Verificação rápida
```bash
ros2 topic echo /telemetry/data --once
ros2 topic hz /telemetry/data
```
