#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
import tf_transformations

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        # Publicador para /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Assinante para o sensor ultrassônico frontal
        self.front_sensor_sub = self.create_subscription(
            PointCloud2, '/front_ultrasonic_plugin/out', self.front_sensor_callback, 10)
        # Assinante para odometria
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        # Variáveis para armazenar as leituras do sensor frontal
        self.front_distance = float('inf')
        self.ground_distance = float('inf')  # Inicializa com infinito
        # Variável para armazenar a orientação atual
        self.current_yaw = 0.0
        self.initial_yaw = None
        # Timer para executar a lógica de movimento
        self.timer = self.create_timer(0.1, self.control_loop)
        # Mensagem Twist para comandos de velocidade
        self.twist = Twist()
        self.get_logger().info('Obstacle Avoidance Node Started')

    def front_sensor_callback(self, msg):
        # Extrair pontos da mensagem PointCloud2
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.get_logger().info(f'Front sensor points: {len(points)}')
        if points:
            # Exibir valores brutos dos pontos para depuração
            self.get_logger().info(f'Raw points: {[point for point in points]}')
            # Calcular a distância mínima (usando a coordenada x)
            distances = [abs(point[0]) for point in points if 0.02 <= abs(point[0]) <= 0.85]  # Reflete o range do URDF
            if distances:
                self.front_distance = min(distances)
                self.get_logger().info(f'Front Distance: {self.front_distance:.2f} m')
                # Definir a distância do chão apenas se for o valor máximo (sem obstáculo)
                if self.ground_distance == float('inf') and self.front_distance == 0.85:
                    self.ground_distance = self.front_distance
                    self.get_logger().info(f'Ground distance set to: {self.ground_distance:.2f} m')
            else:
                self.front_distance = float('inf')
                self.get_logger().info('No valid distances detected by front sensor')
        else:
            self.front_distance = float('inf')
            self.get_logger().info('No points detected by front sensor')

    def odom_callback(self, msg):
        # Extrair a orientação do quaternion
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]  # Yaw (rotação em torno do eixo Z)
        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw
            self.get_logger().info(f'Initial yaw set to: {self.initial_yaw:.2f} radians')

    def control_loop(self):
        # Margem para considerar um obstáculo (20% menor que a distância do chão)
        OBSTACLE_THRESHOLD = self.ground_distance * 0.8 if self.ground_distance != float('inf') else 0.4

        # Log para depuração
        self.get_logger().info(f'Current front distance: {self.front_distance:.2f} m')
        self.get_logger().info(f'Obstacle threshold: {OBSTACLE_THRESHOLD:.2f} m')

        # Lógica de movimento
        if self.front_distance < OBSTACLE_THRESHOLD and self.front_distance != float('inf'):
            # Obstáculo à frente: parar e girar
            self.twist.linear.x = 0.0
            self.twist.angular.z = 2.3  # Girar à esquerda
            self.initial_yaw = None  # Resetar o yaw inicial ao girar intencionalmente
            self.get_logger().info('Obstacle ahead! Turning...')
        else:
            # Sem obstáculos: mover para frente
            self.twist.linear.x = 0.3
            # Correção de desvio angular
            if self.initial_yaw is None:
                self.initial_yaw = self.current_yaw
            if self.initial_yaw is not None:
                yaw_error = self.current_yaw - self.initial_yaw
                Kp = 1.0  # Ganho proporcional
                self.twist.angular.z = -Kp * yaw_error
                self.get_logger().info(f'Moving forward... Yaw error: {yaw_error:.2f}, Correction: {self.twist.angular.z:.2f}')
            else:
                self.twist.angular.z = 0.0
                self.get_logger().info('Moving forward... Waiting for initial yaw')

        # Publicar o comando de velocidade
        self.get_logger().info(f'Publishing cmd_vel: linear.x={self.twist.linear.x:.2f}, angular.z={self.twist.angular.z:.2f}')
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Publicar comando de parada antes de encerrar
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
