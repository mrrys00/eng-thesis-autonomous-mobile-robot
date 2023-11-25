import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import sleep

import serial

# TODO
LINEAR_FACTOR = 1       # 1 real meter = 1.0 /cmd_vel = 25317 robot units
ANGULAR_FACTOR = 1
BASE_SPEED = 10

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

FORWARD = "[o{}]\n"
BACKWARD = "[p{}]\n"
LEFT = "[m{}]\n"
RIGHT = "[n{}]\n"

STOP = "[s]\n"

testing_no_serial = True

class MiaBotNode(Node):
    def __init__(self):
        super().__init__('miabot_node')

        # Subscribe /cmd_vel
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
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
        self.odom_timer = self.create_timer(1.0/31, self.publish_odom)      # 31 odometries / 1 sec

        self.latest_odom = Odometry()

    def cmd_vel_callback(self, msg: Twist):
        """
        Processes /cmd_vel messages to valid odometry type and 
        """
        # Tutaj umieść kod przetwarzający wiadomości na komendy dla robota
        # Poniżej znajduje się przykładowy kod, który publikuje te same dane na temat /odom
        self.get_logger().info(f'Received cmd_vel: Linear={msg.linear.x}, Angular={msg.angular.z}')

        linear = msg.linear.x * LINEAR_FACTOR
        angular = msg.angular.z * ANGULAR_FACTOR

        self.write(
            *self.process_cmd_vel_to_wheels(
                linear,
                angular
            )
        )
        # self.write(
        #     FORWARD.format(linear) if linear > 0 else BACKWARD.format(-linear) +
        #     LEFT.format(angular) if angular > 0 else RIGHT.format(-angular)
        # )

        self.latest_odom = None # Last processed message

    def publish_odom(self):
        """
        Publishes messages to the /odom topic
        """
        # TO DO
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"

        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        odom_msg.twist.twist.linear.x = 1.0
        odom_msg.twist.twist.angular.z = 0.5
        self.odom_publisher.publish(odom_msg)

        self.get_logger().info(f'Odom\nlin x: {odom_msg.twist.twist.linear.x}\n ang z: {odom_msg.twist.twist.angular.z}\n')

    def process_cmd_vel_to_wheels(
        self,
        linear: float,
        angular: float) -> [int, int]:
        """
        Processes recived linear and angular speed to valid robot left and right wheels move.
        """
        # TO DO
        left_factor = 1.0
        right_factor = 1.0

        return BASE_SPEED*1, BASE_SPEED*1

    def write(self, *args):
        """
        Writes data to serial port
        Typically it is left and right speed but it is possible to send custom message
        """
        try:
            left_speed: int = args[0]
            right_speed: int = args[1]
            message: str = f"[l{left_speed},r{right_speed}]\n"
            self.get_logger().info(f'Set speed to {message}')
            self.serial_port.write(message.encode("utf-8"))
            sleep(3)
            self.serial_port.write(STOP.encode("utf-8"))
        except:
            message: str = args[0]
            self.get_logger().warning(f'Sending message: {message}')
            self.serial_port.write(message.encode("utf-8"))

def main(args=None):
    rclpy.init(args=args)

    miabot_node = MiaBotNode()

    rclpy.spin(miabot_node)

    miabot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
