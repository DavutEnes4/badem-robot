import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardListenerSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_listener_subscriber')
        self.get_logger().info("Keyboard listener subscriber initialized.")
        self.subscription = self.create_subscription(
            String,  # Abone olunacak mesaj tipi
            'topic',  # Publisher'ın gönderdiği topic adı
            self.listener_callback,  # Callback fonksiyonu
            10  # Queue size
        )
        self.subscription  # Subscription nesnesi referans olarak korunuyor

    def listener_callback(self, msg):
        # Abone olunduğunda gelen mesajı işleyin
        self.get_logger().info(f'Received: {msg.data}')
        ser = serial.Serial('/dev/ttyUSB0', 9600)
        ser.write(msg.data.encode())  # Gelen mesajı yaz
        ser.close()


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = KeyboardListenerSubscriber()

    rclpy.spin(subscriber_node)  # Subscriber düğümünü çalıştır

    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
