# Monte Carlo Localization — ROS2 + Gazebo

Implementación del algoritmo MCL para localización de robots usando filtro de partículas, ROS2 Humble y Gazebo Classic 11.

## Dependencias

```bash
sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher
pip install "numpy<2" opencv-python pyyaml --break-system-packages
```

## Uso

```bash
# Terminal 1 — Gazebo + robot
ros2 launch mcl_localization mcl.launch.py

# Terminal 2 — Algoritmo MCL
ros2 run mcl_localization mcl_node_loca

# Terminal 3 — Visualización
ros2 run mcl_localization visualizar

# Terminal 4 — Mover el robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Pasos del algoritmo

| Paso | Descripción |
|---|---|
| B | Mapa conocido generado con OpenCV (`maps/generar_mapa.py`) |
| C | Resolución: 0.05 m/px, mapa 200×200 px = 10×10 m |
| D | 300 partículas muestreadas en zonas libres del mapa |
| E | Puntaje por coincidencia de rayos LiDAR con el mapa |
| F | Se conserva el top 25% de partículas |
| G | Dead Reckoning con odometría (`/odom`) |
| H | Partículas se mueven con Δpose + ruido gaussiano |
| I | Se repite cada 500 ms |
