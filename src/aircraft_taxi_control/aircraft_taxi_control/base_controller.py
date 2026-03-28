import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class BaseController(Node):
    """
    Temel sürüş kontrolcüsü.
    Üst katmanlardan (çizgi takibi, güvenlik) gelen Twist mesajlarını
    /aircraft_taxi/cmd_vel topic'ine köprüler.
    Başlatıldığında aracı durdurma mesajı (sıfır hız) yayımlar.
    """

    def __init__(self):
        super().__init__('base_controller')

        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.0)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/aircraft_taxi/cmd_vel',
            10
        )

        # Başlangıçta aracı durdur
        self._publish_stop()

        self.get_logger().info('BaseController başlatıldı — araç durduruldu.')

    def _publish_stop(self):
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)

    def publish_velocity(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
