import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.state         = 'forward'
        self.turn_counter  = 0
        self.turn_steps    = 0
        self.min_front     = 999.0

        # Timer: ejecuta la lógica cada 100ms
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Auto explorer iniciado')

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 999.0)

        # Sectores del LiDAR
        n = len(ranges)
        front_idx = list(range(0, n//8)) + list(range(7*n//8, n))
        left_idx  = list(range(n//8,   3*n//8))
        right_idx = list(range(5*n//8, 7*n//8))

        self.min_front = np.min(ranges[front_idx])
        self.min_left  = np.min(ranges[left_idx])
        self.min_right = np.min(ranges[right_idx])

    def control_loop(self):
        cmd = Twist()

        if self.state == 'forward':
            if self.min_front < 0.6:
                # Obstáculo al frente: decidir hacia donde girar
                self.state = 'turn'
                if self.min_left > self.min_right:
                    self.turn_dir   = 1.0   # izquierda
                else:
                    self.turn_dir   = -1.0  # derecha
                # Gira entre 60 y 120 grados aleatoriamente
                self.turn_steps = int(np.random.uniform(20, 40))
                self.turn_counter = 0
            else:
                cmd.linear.x  = 0.3
                cmd.angular.z = 0.0

        elif self.state == 'turn':
            if self.turn_counter < self.turn_steps:
                cmd.linear.x  = 0.0
                cmd.angular.z = self.turn_dir * 0.5
                self.turn_counter += 1
            else:
                self.state = 'forward'

        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(AutoExplorer())
    rclpy.shutdown()