import rclpy
import serial
import struct
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import String

# TODO
LINEAR_FACTOR = 1
ANGULAR_FACTOR = 1

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

FORWARD = "[o{}]"
BACKWARD = "[p{}]"
LEFT = "[m{}]"
RIGHT = "[n{}]"

class MiaBotNode(Node):
    def __init__(self):
        super().__init__("miabot_node")
        self.cmd_val_subscriber = self.create_subscription(Twist, "cmd_val", self.cmd_val_callback, 10)
        self.odom_publisher = self.create_publisher(Twist, "odom", 10)
        self.output = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    def cmd_val_callback(self, msg: Twist):
        self.get_logger().debug(f"""
                                Received movement command:
                                \n\tLinear:
                                \t{msg.linear.x},
                                \t{msg.linear.y},
                                \t{msg.linear.x},
                                \n\tAngular:
                                \t{msg.angular.x},
                                \t{msg.angular.y},
                                \t{msg.angular.x},
                                \n""")

        linear = msg.linear.x * LINEAR_FACTOR
        angular = msg.angular.z * ANGULAR_FACTOR

        self.write(
            FORWARD.format(linear) if linear > 0 else BACKWARD.format(-linear) +
            LEFT.format(linear) if linear > 0 else RIGHT.format(-linear)
        )

    def write(self, message):
        self.output.write(message.encode("utf-8"))

    def publish_odom(self):
        # Simulate publishing /odom messages similar to turtlebot
        # Your implementation here
        self.odom_publisher.publish(Twist())

    def destroy_node(self):
        self.output.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    miabot_node = MiaBotNode()

    # Implement your node"s logic here (e.g., running a loop to listen to messages and perform actions)

    rclpy.spin(miabot_node)

    # Cleanup
    miabot_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

