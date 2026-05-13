import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import cv2
import yaml
import math

class MapBuilder(Node):
    def __init__(self):
        super().__init__('Map_builder')

        # Parámetros del mapa
        self.resolution = 0.05        # metros por píxel
        self.map_size   = 500         # 500x500 píxeles = 25x25 metros
        self.origin_x   = self.map_size // 2
        self.origin_y   = self.map_size // 2

        # Mapa: 255 = libre, 0 = obstáculo, 128 = desconocido
        self.map_img = np.full((self.map_size, self.map_size), 128, dtype=np.uint8)

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self.odom_cb, 10)

        self.get_logger().info('Map builder iniciado. Mueve el robot para generar el mapa.')
        self.get_logger().info('Presiona S en la ventana de OpenCV para guardar el mapa.')

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = 2 * math.atan2(q.z, q.w)

    def scan_cb(self, msg):
        display = self.map_img.copy()

        # Posición del robot en píxeles
        rx = int(self.origin_x + self.robot_x / self.resolution)
        ry = int(self.origin_y - self.robot_y / self.resolution)

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment + self.robot_yaw

            if r < msg.range_min or r > msg.range_max:
                continue

            # Punto final del rayo (obstáculo)
            ex = self.robot_x + r * math.cos(angle)
            ey = self.robot_y + r * math.sin(angle)

            px = int(self.origin_x + ex / self.resolution)
            py = int(self.origin_y - ey / self.resolution)

            # Marcar rayo como libre (bresenham)
            self.draw_free_ray(rx, ry, px, py)

            # Marcar obstáculo
            if 0 <= px < self.map_size and 0 <= py < self.map_size:
                self.map_img[py, px] = 0

        # Dibujar robot en la visualización
        cv2.circle(display, (rx, ry), 4, (180, 180, 180), -1)

        cv2.imshow('Mapa MCL (S=guardar, Q=salir)', display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self.save_map()
        elif key == ord('q'):
            self.save_map()
            rclpy.shutdown()

    def draw_free_ray(self, x0, y0, x1, y1):
        # Algoritmo de Bresenham para marcar píxeles libres a lo largo del rayo
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while True:
            if 0 <= x < self.map_size and 0 <= y < self.map_size:
                if self.map_img[y, x] == 128:
                    self.map_img[y, x] = 255  # libre
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def save_map(self):
        path = '/home/alondra/ros2_ws/src/mcl_robot/maps/mi_mapa'
        cv2.imwrite(path + '.pgm', self.map_img)

        meta = {
            'image':      'mi_mapa.pgm',
            'resolution': self.resolution,
            'origin':     [
                -(self.origin_x * self.resolution),
                -(self.origin_y * self.resolution),
                0.0
            ],
            'negate':      0,
            'occupied_thresh': 0.65,
            'free_thresh':     0.196
        }
        with open(path + '.yaml', 'w') as f:
            yaml.dump(meta, f)

        self.get_logger().info(f'Mapa guardado en {path}.pgm y {path}.yaml')

def main():
    rclpy.init()
    rclpy.spin(MapBuilder())
    rclpy.shutdown()