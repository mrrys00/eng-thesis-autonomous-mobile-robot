import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from time import sleep, time_ns

from math import pi, sin, cos

import serial

# TODO
LINEAR_FACTOR = 500         # 1 real meter = 1.0 /cmd_vel = 25317 robot units
DELTA_RADIUS = 0.03         # distance between singe wheel and robot center
ODOM_FREQUENCY = 2**4       # odometry messages per second -> optimal 2**4 (?)

MAX_SAFE_VELOCITY = 40
# ANGULAR_FACTOR = 1


SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

# False if Miabot connected
testing_no_serial = False

class MiaBotNode(Node):
    def __init__(self):
        super().__init__('miabot_node')

        # self.init_time = time_ns()

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
        self.tf_publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10)
        self.publish_timer = self.create_timer(1.0 / ODOM_FREQUENCY, self.publish_odom_tf)

        def _init_transform() -> TransformStamped:
            transform = TransformStamped()
            transform.header.frame_id = 'odom'
            transform.header.stamp = self.get_clock().now().to_msg()

            transform.child_frame_id = 'base_footprint'
            
            # actual position
            transform.transform.translation.x = 0.0     # actual x
            transform.transform.translation.y = 0.0     # actual y
            transform.transform.translation.z = 0.0     # always 0

            # actual rotation
            transform.transform.rotation.x = 0.0        # always 0
            transform.transform.rotation.y = 0.0        # always 0
            transform.transform.rotation.z = 0.0        # actual radian rotation
            transform.transform.rotation.w = 1.0        # always 1.0

            return transform

        def _init_odometry() -> Odometry:
            odom_msg = Odometry()
            odom_msg.header.frame_id = "odom"
            odom_msg.header.stamp = self.get_clock().now().to_msg()

            odom_msg.child_frame_id = "base_footprint"

            odom_msg.pose.pose.position.x = 0.0     # actual x
            odom_msg.pose.pose.position.y = 0.0     # actual y
            odom_msg.pose.pose.position.z = 0.0     # always 0

            odom_msg.pose.pose.orientation.x = 0.0  # always 0
            odom_msg.pose.pose.orientation.y = 0.0  # always 0
            odom_msg.pose.pose.orientation.z = 0.0  # actual radian rotation
            odom_msg.pose.pose.orientation.w = 1.0  # always 1.0

            odom_msg.twist.twist.linear.x = 0.0     # linear velocity
            odom_msg.twist.twist.linear.y = 0.0     # always 0
            odom_msg.twist.twist.linear.z = 0.0     # always 0

            odom_msg.twist.twist.angular.x = 0.0    # always 0
            odom_msg.twist.twist.angular.y = 0.0    # always 0
            odom_msg.twist.twist.angular.z = 0.0    # angular valocity

            return odom_msg

        self.odom_msg = _init_odometry()
        self.transform_msg = _init_transform()


    def cmd_vel_callback(self, msg: Twist):
        """
        Processes /cmd_vel messages to valid odometry type and 
        """
        self.get_logger().info(f'Received cmd_vel: Linear={msg.linear.x}, Angular={msg.angular.z}')

        linear = msg.linear.x
        angular = msg.angular.z

        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular

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

    def update_robot_pose_info(self, duration: float):
        """
        Calculates data to /tf and /odom topics 
        """
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()

        x_linear = self.odom_msg.twist.twist.linear.x
        z_angular = self.odom_msg.twist.twist.angular.z
        z_oriantation = self.odom_msg.pose.pose.orientation.z

        self.odom_msg.pose.pose.position.x += cos(z_oriantation) * (x_linear*duration)      # actual x 2d
        self.odom_msg.pose.pose.position.y += sin(z_oriantation) * (x_linear*duration)      # actual y 2d

        self.odom_msg.pose.pose.orientation.z += (z_angular*duration) % (2*pi)              # actual z rotation

        self.transform_msg.transform.translation.x = self.odom_msg.pose.pose.position.x     # actual x 2d
        self.transform_msg.transform.translation.y = self.odom_msg.pose.pose.position.y     # actual y 2d

        self.transform_msg.transform.rotation.z = self.odom_msg.pose.pose.orientation.z     # actual z rotation

    def publish_odom_tf(self):
        """
        Publishes messages to the /odom topic
        """
        self.update_robot_pose_info(1.0 / ODOM_FREQUENCY)

        tf_message = TFMessage()
        tf_message.transforms.append(self.transform_msg)

        self.odom_publisher.publish(self.odom_msg)      # frame_id: odom; child_frame_id: base_footprint
        self.tf_publisher.publish(tf_message)   # frame_id: odom; child_frame_id: base_footprint
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

        if v_left > MAX_SAFE_VELOCITY or v_right > MAX_SAFE_VELOCITY:
            self.get_logger().warning(f"Safe velocity exceeded:\n\tv_left: {v_left}\n\tv_right: {v_right}")

        v_left, v_right = min(v_left, MAX_SAFE_VELOCITY), min(v_right, MAX_SAFE_VELOCITY)

        return int(v_left), int(v_right)

    def msg_velocity(self, v_left: int, v_right: int) -> str:
        return f"[=<{v_left}l>,<{v_right}r>]\n"

    def write(self, msg: str):
        """
        Writes data to serial port.
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
