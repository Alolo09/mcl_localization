import os
import math
import threading

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

# Constantes visuales 
WIN_NAME      = "MCL – Partículas"
PARTICLE_R    = 3        # radio del punto de partícula (px)
BEST_R        = 7        # radio de la estimación mejor
COLOR_PART    = (0, 0, 220)   # rojo (BGR)
COLOR_BEST    = (0, 210, 0)   # verde
COLOR_ELLIPSE = (210, 80, 0)  # azul
COLOR_TEXT    = (255, 255, 255)
SCALE         = 2        # factor de zoom para que el mapa no sea diminuto

MAP_BASE = os.path.expanduser('~/ros2_ws/src/mcl_localization/maps')
MAP_PGM  = os.path.join(MAP_BASE, 'mi_mapa.pgm')
MAP_YAML = os.path.join(MAP_BASE, 'mi_mapa.yaml')


# Nodo visualizador 
class VisualizarParticulas(Node):

    def __init__(self):
        super().__init__('visualizar_particulas')

        # Cargar mapa
        self._load_map()

        # Estado compartido entre el callback ROS y el hilo de visualización
        self._lock       = threading.Lock()
        self._particles  = []   # lista de (x, y, th) en coordenadas mundo
        self._new_data   = False

        # Subscripción al topic de partículas
        self.create_subscription(PoseArray, '/particles', self._particles_cb, 10)
        self.get_logger().info(
            f'Visualizador listo. Escuchando /particles … '
            f'[mapa {self._map_w}×{self._map_h} px, res={self._resolution} m/px]'
        )

    #  Carga del mapa 
    def _load_map(self):
        if not os.path.isfile(MAP_PGM):
            self.get_logger().error(f'No se encontró el mapa: {MAP_PGM}')
            raise FileNotFoundError(MAP_PGM)

        if not os.path.isfile(MAP_YAML):
            self.get_logger().error(f'No se encontró el YAML: {MAP_YAML}')
            raise FileNotFoundError(MAP_YAML)

        with open(MAP_YAML) as f:
            meta = yaml.safe_load(f)

        self._resolution = meta['resolution']          # m / px
        self._origin     = meta['origin']              # [x0, y0, theta]

        raw = cv2.imread(MAP_PGM, cv2.IMREAD_GRAYSCALE)
        if raw is None:
            raise RuntimeError(f'cv2.imread no pudo leer {MAP_PGM}')

        self._map_h, self._map_w = raw.shape

        # Convertir a BGR y escalar para visualización
        bgr = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        self._map_base = cv2.resize(
            bgr,
            (self._map_w * SCALE, self._map_h * SCALE),
            interpolation=cv2.INTER_NEAREST
        )

    # Conversión coordenadas mundo → píxel
    def _world_to_px(self, x, y):
        px = int((x - self._origin[0]) / self._resolution) * SCALE
        # El eje Y del mapa está invertido respecto al mundo ROS
        py = int((y - self._origin[1]) / self._resolution) * SCALE
        return px, py

    # Callback: llegaron nuevas partículas
    def _particles_cb(self, msg: PoseArray):
        pts = []
        for pose in msg.poses:
            x  = pose.position.x
            y  = pose.position.y
            q  = pose.orientation
            th = 2 * math.atan2(q.z, q.w)
            pts.append((x, y, th))

        with self._lock:
            self._particles = pts
            self._new_data  = True

    # Dibujar frame 
    def draw(self):
        with self._lock:
            if not self._new_data:
                return None
            pts      = list(self._particles)
            self._new_data = False

        if not pts:
            return None

        frame = self._map_base.copy()
        h, w  = frame.shape[:2]

        xs  = np.array([p[0] for p in pts])
        ys  = np.array([p[1] for p in pts])

        # Dibujar puntos de partículas 
        for x, y, _ in pts:
            px, py = self._world_to_px(x, y)
            if 0 <= px < w and 0 <= py < h:
                cv2.circle(frame, (px, py), PARTICLE_R, COLOR_PART, -1,
                           lineType=cv2.LINE_AA)

        # Elipse de dispersión 
        std_x = float(np.std(xs))
        std_y = float(np.std(ys))
        cx, cy = self._world_to_px(float(np.mean(xs)), float(np.mean(ys)))

        ax = max(1, int(std_x / self._resolution) * SCALE)
        ay = max(1, int(std_y / self._resolution) * SCALE)

        if 0 <= cx < w and 0 <= cy < h:
            cv2.ellipse(frame, (cx, cy), (ax, ay), 0, 0, 360,
                        COLOR_ELLIPSE, 2, lineType=cv2.LINE_AA)

            # Mejor estimación (ni tanto)
            cv2.circle(frame, (cx, cy), BEST_R, COLOR_BEST, -1,
                       lineType=cv2.LINE_AA)
            cv2.circle(frame, (cx, cy), BEST_R, (0, 0, 0), 1,
                       lineType=cv2.LINE_AA)

        # HUD de texto
        dispersión = math.sqrt(std_x**2 + std_y**2)
        lines = [
            f'Particulas : {len(pts)}',
            f'Estimacion : ({np.mean(xs):.2f}, {np.mean(ys):.2f}) m',
            f'Dispersion : {dispersión:.3f} m  (sx={std_x:.3f} sy={std_y:.3f})',
        ]
        font  = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thick = 1
        pad   = 8
        lh    = 22
        for i, line in enumerate(lines):
            ypos = pad + (i + 1) * lh
            # sombra
            cv2.putText(frame, line, (pad + 1, ypos + 1),
                        font, scale, (0, 0, 0), thick + 1, cv2.LINE_AA)
            cv2.putText(frame, line, (pad, ypos),
                        font, scale, COLOR_TEXT, thick, cv2.LINE_AA)

        return frame


# Bucle principal 
def main(args=None):
    rclpy.init(args=args)
    node = VisualizarParticulas()

    # Hilo ROS en background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_NAME, 800, 600)

    node.get_logger().info(
        'Ventana OpenCV abierta. Presiona Q o ESC para salir.'
    )

    try:
        while rclpy.ok():
            frame = node.draw()
            if frame is not None:
                cv2.imshow(WIN_NAME, frame)

            key = cv2.waitKey(100) & 0xFF   # refresca a ~10 Hz
            if key in (ord('q'), ord('Q'), 27):   # Q o ESC
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
