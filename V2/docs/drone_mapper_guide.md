# drone_mapper Guide (V2)

## Objetivo do pacote

O `drone_mapper` pega telemetria em `/telemetry/data`, converte para posição no frame `map` e desenha trajetória/pose sobre o mapa 3D.

## Nós do pacote

| Nó | Papel | Essencial? |
|---|---|---|
| `/map_publisher` | Publica o mapa PCD em `/map/cloud` (latched) | **Sim** |
| `/drone_localization_node` | Converte telemetria para pose/path/TF/status | **Sim** |

## Tópicos do pacote (publicados)

| Tópico | Tipo | Papel | Essencial? | Se remover |
|---|---|---|---|---|
| `/map/cloud` | `sensor_msgs/PointCloud2` | Mapa 3D para RViz | **Sim** | Sem mapa |
| `/map/telemetry_pose` | `geometry_msgs/PoseStamped` | Pose atual do drone | **Sim** | Sem indicador de posição atual |
| `/map/telemetry_path` | `nav_msgs/Path` | Trajetória histórica | **Sim** | Sem linha principal da rota |
| `/tf` (`map -> drone_reference`) | `tf2` | Referencial dinâmico do drone | **Sim** | Pose/axes no RViz e integração TF pioram |
| `/map/telemetry_odometry` | `nav_msgs/Odometry` | Compatibilidade com consumidores de odometria | Opcional | Só perde integração com nós que exigem odom |
| `/map/telemetry_status` | `std_msgs/String` | Diagnóstico (accepted/rejected) | Opcional (recomendado) | Debug fica difícil |

## Tópicos que o pacote consome

| Tópico | Papel | Essencial? |
|---|---|---|
| `/telemetry/data` | Entrada de latitude/longitude/heading/height | **Sim** |

## Mínimo do mínimo (KISS)

Para manter somente o necessário para operação visual:
- **Manter:** `/map/cloud`, `/map/telemetry_pose`, `/map/telemetry_path`, `/tf`, `/telemetry/data`
- **Opcional/removível:** `/map/telemetry_odometry`, `/map/telemetry_status`

## Comandos

### Build (Docker)
```bash
cd /workspace/V2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drone_mapper --symlink-install
source install/setup.bash
```

### Run
```bash
ros2 launch drone_mapper drone_mapper.launch.py config_file:=/workspace/V2/config.yaml
```

### RViz
```bash
rviz2 -d /workspace/V2/src/drone_mapper/rviz/drone_mapper.rviz
```

## Verificação rápida
```bash
ros2 node list
ros2 topic list
ros2 topic echo /map/telemetry_status --once
ros2 topic echo /map/telemetry_pose --once
ros2 topic echo /map/telemetry_path --once
```
