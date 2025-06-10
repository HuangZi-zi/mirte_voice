import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
import subprocess
import time

class SoundPlayer(Node):
    def __init__(self):
        super().__init__('sound_player')
        self.get_logger().info("Receiver Ready!")

        # Subscription to trigger external sound play
        self.subscription = self.create_subscription(
            String,
            '/play_sound',
            self.play_callback,
            10
        )

        # Subscription to battery info
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/io/power/power_watcher',
            self.battery_callback,
            10
        )
        self.latest_voltage = None
        # fully charged 12.6v; fully discharged 9v
        self.low_voltage_threshold = 0.2 # percentage
        self.last_warning_time = 0
        self.warning_interval = 5  # seconds

        # Timer to check battery status
        self.timer = self.create_timer(5.0, self.check_battery_status)

    def play_callback(self, msg):
        self.play_sound(msg.data)

    def battery_callback(self, msg: BatteryState):
        self.latest_voltage = msg.voltage

    def check_battery_status(self):
        if self.latest_voltage is None:
            return
        percentage = (self.latest_voltage-9.0)/3.6
        if percentage < self.low_voltage_threshold:
            now = time.time()
            if now - self.last_warning_time > self.warning_interval:
                self.get_logger().warn(f"Low voltage detected: {self.latest_voltage:.2f}V")
                self.play_sound('/home/mirte/mirte_voice/low_battery.mp3')
                self.last_warning_time = now

    def play_sound(self, sound_path):
        self.get_logger().info(f"Playing sound: {sound_path}")
        try:
            subprocess.run(
                ['ffplay', '-nodisp', '-autoexit', sound_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            self.get_logger().error(f"Failed to play sound: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SoundPlayer()
    rclpy.spin(node)
    rclpy.shutdown()


