import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.get_logger().info('Obstacle Avoidance Node Started')
        self.front_sub = self.create_subscription(PointCloud2, '/front_ultrasonic_plugin/out', self.front_callback, 10)
        self.rear_sub = self.create_subscription(PointCloud2, '/rear_ultrasonic_plugin/out', self.rear_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front_range = 4.0
        self.rear_range = 4.0
        self.timer = self.create_timer(0.1, self.control_loop)

    def front_callback(self, msg):
        # Extrair pontos da mensagem PointCloud2
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if points:
            # Calcular a distância mínima (usando apenas a coordenada x, já que o sensor aponta para frente)
            distances = [point[0] for point in points]
            self.front_range = min(distances)
            self.get_logger().info(f'Front range: {self.front_range:.2f} m')
        else:
            self.front_range = 4.0  # Máximo alcance se não houver pontos
            self.get_logger().info('No points detected by front sensor')

    def rear_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if points:
            distances = [point[0] for point in points]
            self.rear_range = min(distances)
            self.get_logger().info(f'Rear range: {self.rear_range:.2f} m')
        else:
            self.rear_range = 4.0
            self.get_logger().info('No points detected by rear sensor')

    def control_loop(self):
        msg = Twist()
        if self.front_range < 0.5 or self.rear_range < 0.5:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Obstacle detected! Stopping...')
        else:
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.get_logger().info('Moving forward...')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
