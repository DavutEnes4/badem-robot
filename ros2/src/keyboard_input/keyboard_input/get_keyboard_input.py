import rclpy
from rclpy.node import Node
import sys
import termios
import tty
from std_msgs.msg import String

class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        self.get_logger().info("Keyboard listener initialized. Press any key...")
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.listen_keyboard)
        self.i = 0

    def listen_keyboard(self):
        # Terminal ayarlarını değiştir
        old_settings = termios.tcgetattr(sys.stdin)
        

        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)  # Tek bir karakter oku
                print(type(key))
                self.get_logger().info(f'Key pressed: {key}')
                
                msg = String()
                msg.data = f'{key}'
                self.publisher_.publish(msg)
                
                if key.lower() == 'q':  # 'q' tuşuna basıldığında çıkış yap
                    self.get_logger().info("Exiting...")
                    break
                
        finally:
            # Eski terminal ayarlarını geri yükle
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardListener()
    try:
        node.listen_keyboard()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
