import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from time import sleep, time_ns

from copy import deepcopy

from math import pi, sin, cos

import serial
import tf2_ros

# TODO
LINEAR_FACTOR = 500         # 1 real meter = 1.0 /cmd_vel = 25317 robot units
DELTA_RADIUS = 0.03         # distance between singe wheel and robot center
ODOM_FREQUENCY = 2**5       # odometry messages per second -> optimal 2**5 (minimum 20Hz)

MAX_SAFE_VELOCITY = 120
ANGULAR_FACTOR = 225/270    # experimental; only for odometry purposes :P


SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

# False if Miabot connected
testing_no_serial = False

class MiaBotNode(Node):
    def __init__(self):
        super().__init__('miabot_node')

        # TO DO
        # - change mapping base_link to base footprint 
        # - base link to odom 

        # /imu_link - publishes frame_id imu_link orientation and lin and ang velocity
        # /pose - published by slam (?)
        # /plan - without frame_id but full position 


        self.update_frequency = 1.0 / ODOM_FREQUENCY
        # self.static_update_frequency = float(ODOM_FREQUENCY)

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
        # self.pose_publisher = self.create_publisher(
        #     PoseStamped,
        #     '/pose',
        #     10)
        # self.tf_static_publisher = self.create_publisher(
        #     TFMessage,
        #     '/tf_static',
        #     10)

        self.odom_publish_timer = self.create_timer(self.update_frequency, self.publish_odom)
        # self.tf_static_publish_timer = self.create_timer(self.static_update_frequency, self.publish_tf_static)
        self.tf_publish_timer = self.create_timer(self.update_frequency, self.publish_tf)
        # self.pose_publish_timer = self.create_timer(self.update_frequency, self.publish_pose)

        def _init_transform_odom_base_footprint() -> TransformStamped:
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

        def _init_transform_odom_base_link(tf_odom_base_ftpr: TransformStamped) -> TransformStamped:
            tf_odom_base_ftpr_copy = deepcopy(tf_odom_base_ftpr)
            tf_odom_base_ftpr_copy.child_frame_id = 'base_link'
            return tf_odom_base_ftpr_copy

        # def _init_transform_base_foootprint_base_link(tf_odom_base_link: TransformStamped) -> TransformStamped:
        #     tf_odom_base_link.header.frame_id = 'base_footprint'
        #     return tf_odom_base_link

        def _init_odometry() -> Odometry:       # as in turtlebot - OK
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
        
        # def _init_pose() -> PoseStamped:
        #     pose_msg = PoseStamped()
        #     pose_msg.header.frame_id = 'map'
        #     pose_msg.header.stamp = self.get_clock().now().to_msg()

        #     pose_msg.pose.position.x = 0.0
        #     pose_msg.pose.position.y = 0.0
        #     pose_msg.pose.position.z = 0.0
            
        #     pose_msg.pose.orientation.x = 0.0
        #     pose_msg.pose.orientation.y = 0.0
        #     pose_msg.pose.orientation.z = 0.0
        #     pose_msg.pose.orientation.w = 1.0

        #     return pose_msg

        self.odom_msg = _init_odometry()
        self.transform_msg_odom_base_footprint = _init_transform_odom_base_footprint()
        self.transform_msg_odom_base_link = _init_transform_odom_base_link(self.transform_msg_odom_base_footprint)      # unnecessary ?
        # self.transform_msg_base_footprint_base_link = _init_transform_base_foootprint_base_link(self.transform_msg_odom_base_link)
        # self.pose_msg = _init_pose()        # provided by slam toolbox


    def cmd_vel_callback(self, msg: Twist):
        """
        Processes /cmd_vel messages to valid odometry type and 
        """
        self.get_logger().info(f'Received cmd_vel: Linear={msg.linear.x}, Angular={msg.angular.z}')

        linear = msg.linear.x
        angular = msg.angular.z

        v_left, v_right, real_linear, real_angular = self.process_cmd_vel_to_wheels(
            linear,
            angular
        )

        self.odom_msg.twist.twist.linear.x = real_linear
        self.odom_msg.twist.twist.angular.z = real_angular
        
        self.write(
            self.msg_velocity(
                v_left,
                v_right
            )
        )

    def update_robot_pose_info(self):
        """
        Calculates data to /tf and /odom topics 
        """
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.transform_msg_odom_base_footprint.header.stamp = self.get_clock().now().to_msg()
        self.transform_msg_odom_base_link.header.stamp = self.get_clock().now().to_msg()

        x_linear = self.odom_msg.twist.twist.linear.x
        z_angular = self.odom_msg.twist.twist.angular.z
        z_oriantation = self.odom_msg.pose.pose.orientation.z

        self.odom_msg.pose.pose.position.x += cos(z_oriantation) * (x_linear*self.update_frequency)         # actual x 2d
        self.odom_msg.pose.pose.position.y += sin(z_oriantation) * (x_linear*self.update_frequency)         # actual y 2d

        self.odom_msg.pose.pose.orientation.z += (z_angular*self.update_frequency)                          # actual z rotation
        self.odom_msg.pose.pose.orientation.z = (self.odom_msg.pose.pose.orientation.z + pi) % (2 * pi) - pi

        self.transform_msg_odom_base_footprint.transform.translation.x = self.odom_msg.pose.pose.position.x     # actual x 2d
        self.transform_msg_odom_base_link.transform.translation.x = self.odom_msg.pose.pose.position.x     # actual x 2d
        self.transform_msg_odom_base_footprint.transform.translation.y = self.odom_msg.pose.pose.position.y     # actual y 2d
        self.transform_msg_odom_base_link.transform.translation.y = self.odom_msg.pose.pose.position.y     # actual y 2d

        self.transform_msg_odom_base_footprint.transform.rotation.z = self.odom_msg.pose.pose.orientation.z     # actual z rotation
        self.transform_msg_odom_base_link.transform.rotation.z = self.odom_msg.pose.pose.orientation.z     # actual z rotation

    def publish_odom(self):
        """
        Publishes messages to the /odom topic
        """
        self.update_robot_pose_info()
        self.odom_publisher.publish(self.odom_msg)      # frame_id: odom; child_frame_id: base_footprint
        # self.tf_publisher.publish(tf_message)   # frame_id: odom; child_frame_id: base_footprint  # potentially produces errors :')
        # self.tf_static_publisher.sendTransform(self.transform_msg_odom_base_footprint)
        # self.get_logger().info(f'Odom\nlin x: {self.odom_msg.twist.twist.linear.x}\n ang z: {self.odom_msg.twist.twist.angular.z}')

    # def publish_tf_static(self):
    #     """
    #     Publishes messages to /tf_static topic
    #     """
    #     static_tf_message = TFMessage()
    #     static_tf_message.transforms.append(self.transform_msg_odom_base_footprint)
    #     self.tf_static_publisher.publish(static_tf_message)

    def publish_tf(self):
        """
        Publishes messages to /tf topic
        """
        tfm = TFMessage()
        # tfm.transforms.append(self.transform_msg_odom_base_footprint)
        tfm.transforms.append(self.transform_msg_odom_base_link)      # unnecessary ?
        self.tf_publisher.publish(tfm)

    # def publish_pose(self):
    #     """
    #     Publishes messages to /pose topic
    #     """
    #     self.pose_publisher.publish(self.pose_msg)
        # self.get_logger().info(f'Odom\nlin x: {self.odom_msg.twist.twist.linear.x}\n ang z: {self.odom_msg.twist.twist.angular.z}')

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

        # if v_left > MAX_SAFE_VELOCITY or v_right > MAX_SAFE_VELOCITY:
        #     self.get_logger().warning(f"Safe velocity exceeded:\n\tv_left: {v_left}\n\tv_right: {v_right}")

        v_left, v_right = min(v_left, MAX_SAFE_VELOCITY), min(v_right, MAX_SAFE_VELOCITY)
        v_left, v_right = int(v_left), int(v_right)

        # we need to reproduce real linear and angular velopcities for odometry
        real_linear = (v_left + v_right) / (2 * LINEAR_FACTOR)
        real_angular = (v_right - v_left) / (2 * LINEAR_FACTOR * DELTA_RADIUS)

        return v_left, v_right, real_linear, real_angular

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
