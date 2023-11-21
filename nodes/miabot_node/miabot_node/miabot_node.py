import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial

# TODO
LINEAR_FACTOR = 1
ANGULAR_FACTOR = 1

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

FORWARD = "[o{}]"
BACKWARD = "[p{}]"
LEFT = "[m{}]"
RIGHT = "[n{}]"

testing_no_serial = True

class MiaBotNode(Node):
    def __init__(self):
        super().__init__('miabot_node')

        # Subscribe /cmd_val
        self.cmd_val_subscription = self.create_subscription(
            Twist,
            '/cmd_val',
            self.cmd_val_callback,
            10)

        # Miabot serial init
        self.serial_port = None
        if not testing_no_serial:
            self.serial_port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


        # /odom publisher init
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10)
        self.odom_timer = self.create_timer(1.0, self.publish_odom) # check timer

        self.latest_odom = Odometry()

    def cmd_val_callback(self, msg):
        # Process cmd_val -> Odometry()
        # Tutaj umieść kod przetwarzający wiadomości na komendy dla robota
        # Poniżej znajduje się przykładowy kod, który publikuje te same dane na temat /odom
        self.get_logger().info(f'Received cmd_val: Linear={msg.linear.x}, Angular={msg.angular.z}')

        self.write(
            FORWARD.format(linear) if linear > 0 else BACKWARD.format(-linear) +
            LEFT.format(linear) if linear > 0 else RIGHT.format(-linear)
        )

        self.latest_odom = None # Last processed message

    def publish_odom(self):
        # publish /odom
        odom_msg = Odometry()
        odom_msg.twist.twist.linear.x = 1.0  # example
        odom_msg.twist.twist.angular.z = 0.5  # example
        self.odom_publisher.publish(odom_msg)

    def write(self, message):
        self.output.write(message.encode("utf-8"))

def main(args=None):
    rclpy.init(args=args)

    miabot_node = MiaBotNode()

    rclpy.spin(miabot_node)

    miabot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
