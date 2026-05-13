import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import math
import cv2
import os
from .Particle_filter import ParticleFilter

class MCLNode(Node):
    def __init__(self):
        super().__init__('mcl_node')

        # Rutas al mapa
        base = os.path.expanduser(
            '~/ros2_ws/src/mcl_localization/maps')
        map_pgm  = os.path.join(base, 'mi_mapa.pgm')
        map_yaml = os.path.join(base, 'mi_mapa.yaml')

        self.pf        = ParticleFilter(map_pgm, map_yaml, n_particles=300)
        self.prev_odom = None
        self.scan_data = None

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(PoseArray, '/particles', 10)

        # Loop MCL cada 500ms
        self.create_timer(0.5, self.mcl_loop)
        self.get_logger().info('MCL Node iniciado')

    def scan_cb(self, msg):
        self.scan_data = msg

    def odom_cb(self, msg):
        pose = msg.pose.pose
        x  = pose.position.x
        y  = pose.position.y
        q  = pose.orientation
        th = 2 * math.atan2(q.z, q.w)

        if self.prev_odom is None:
            self.prev_odom = (x, y, th)
            return

        px, py, pth = self.prev_odom
        dx  = x  - px
        dy  = y  - py
        dth = th - pth
        self.prev_odom = (x, y, th)

        # G + H: mover partículas con dead reckoning
        self.pf.move_particles(dx, dy, dth)

    def mcl_loop(self):
        if self.scan_data is None:
            return

        msg = self.scan_data

        # E: puntajes
        scores = self.pf.score_particles(
            msg.ranges, msg.angle_min, msg.angle_increment)

        # F: filtrar y resamplear
        self.pf.resample(scores)

        # Publicar partículas para visualización
        self.publish_particles()

        # Log de la mejor estimación
        est = self.pf.best_estimate()
        self.get_logger().info(
            f'Pose estimada → x:{est[0]:.2f} y:{est[1]:.2f} θ:{math.degrees(est[2]):.1f}°')

    def publish_particles(self):
        out = PoseArray()
        out.header.frame_id = 'map'
        out.header.stamp    = self.get_clock().now().to_msg()
        for x, y, th in self.pf.particles:
            p = Pose()
            p.position.x    = x
            p.position.y    = y
            p.orientation.z = math.sin(th / 2)
            p.orientation.w = math.cos(th / 2)
            out.poses.append(p)
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(MCLNode())
    rclpy.shutdown()