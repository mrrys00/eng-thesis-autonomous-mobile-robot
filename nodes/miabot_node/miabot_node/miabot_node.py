import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import sleep

import serial

# TODO
LINEAR_FACTOR = 500         # 1 real meter = 1.0 /cmd_vel = 25317 robot units
DELTA_RADIUS = 0.03         # distance between singe wheel and robot center
ODOM_FREQUENCY = 2**1       # odometry messages per second -> optimal 2**4 (?)
# ANGULAR_FACTOR = 1


SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

# False if Miabot connected
testing_no_serial = False

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
        self.odom_timer = self.create_timer(1.0 / ODOM_FREQUENCY, self.publish_odom)

        def _init_odometry() -> Odometry:
            odom_msg = Odometry()
            odom_msg.header.frame_id = "odom"

            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0

            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = 0.0
            odom_msg.pose.pose.orientation.w = 1.0

            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.angular.z = 0.0

            return odom_msg

        self.odom_msg = _init_odometry()


    def cmd_vel_callback(self, msg: Twist):
        """
        Processes /cmd_vel messages to valid odometry type and 
        """
        self.get_logger().info(f'Received cmd_vel: Linear={msg.linear.x}, Angular={msg.angular.z}')

        linear = msg.linear.x
        angular = msg.angular.z

        self.update_robot_data(linear, angular)

        v_left, v_right = self.process_cmd_vel_to_wheels(
            linear,
            angular
        )
        self.write(
            self.msg_velocity(
                v_left,
                v_right
            )
        )

    def update_robot_data(
        self,
        linear: float,
        angular: float):
        # TO DO
        # ds = linear
        self.odom_msg = self.odom_msg

    def publish_odom(self):
        """
        Publishes messages to the /odom topic
        """
        self.odom_publisher.publish(self.odom_msg)
        self.get_logger().info(f'Odom\nlin x: {self.odom_msg.twist.twist.linear.x}\n ang z: {self.odom_msg.twist.twist.angular.z}')

    def process_cmd_vel_to_wheels(
        self,
        linear: float,
        angular: float):
        """
        Processes recived linear and angular speed to valid robot left and right wheels move.
        """
        # TO DO - not suer if that works properly
        v_left = LINEAR_FACTOR * (linear - angular * DELTA_RADIUS)
        v_right = LINEAR_FACTOR * (linear + angular * DELTA_RADIUS)

        return int(v_left), int(v_right)

    def msg_velocity(self, v_left: int, v_right: int) -> str:
        return f"[=<{v_left}l>,<{v_right}r>]\n"

    def write(self, msg: str):
        """
        Writes data to serial port
        Typically it is left and right speed but it is possible to send custom message
        """
        try:
            self.serial_port.write(msg.encode('utf-8'))
            self.get_logger().info(f'Set speed to {msg}')
        except Exception as err:
            self.get_logger().warning(f"Unexpected {err=}, {type(err)=}")

def main(args=None):
    rclpy.init(args=args)

    miabot_node = MiaBotNode()

    rclpy.spin(miabot_node)

    miabot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
