import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class SoundPlayer(Node):
    def __init__(self):
        super().__init__('sound_player')
        self.get_logger().info("Receiver Ready!")
        self.subscription = self.create_subscription(
            String,
            '/play_sound',
            self.play_callback,
            10
        )

    def play_callback(self, msg):
        sound_path = msg.data
        self.get_logger().info(f"Playing sound: {sound_path}")
        try:
            subprocess.run(
                ['ffplay', '-nodisp', '-autoexit', sound_path],
                # stdout=subprocess.DEVNULL,
                # stderr=subprocess.DEVNULL
            )
        except Exception as e:
            self.get_logger().error(f"Failed to play sound: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SoundPlayer()
    rclpy.spin(node)
    rclpy.shutdown()

