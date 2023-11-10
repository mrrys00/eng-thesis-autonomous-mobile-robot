import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class ExplorationAlgorithm(Node):

    def __init__(self):
        super().__init__('exploration_algorithm')
        self.get_logger().info('Exploration algorithm is running')
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.navigation_client = self.create_client(NavigateToPose, 'navigate_to_pose')
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_pose = PoseStamped()
        while not self.navigation_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for navigation server...')

    def map_callback(self, msg):
        # Przetwarzaj mapę
        unexplored_points = self.find_unexplored_points(msg)
        if unexplored_points:
            # Jeśli znaleziono nieodwiedzone punkty, planuj trasę do pierwszego z nich
            self.goal_pose.header = Header()
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.pose.position.x = unexplored_points[0][0]
            self.goal_pose.pose.position.y = unexplored_points[0][1]
            self.goal_pose.pose.orientation.w = 1.0

            self.send_navigation_goal(self.goal_pose)
        else:
            # Jeśli nie ma nieodwiedzonych punktów, wysyłamy cel (0,0,0) jako punkt docelowy
            self.goal_pose.header = Header()
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.pose.position.x = 0.0
            self.goal_pose.pose.position.y = 0.0
            self.goal_pose.pose.orientation.w = 1.0

            self.send_navigation_goal(self.goal_pose)
            self.get_logger().info('Exploration complete')

    def find_unexplored_points(self, map):
        unexplored_points = []
        map_data = map.data
        map_width = map.info.width
        map_height = map.info.height

        for i in range(map_width):
            for j in range(map_height):
                if map_data[i + j * map_width] == -1:
                    # Oznacza to nieznaną pozycję na mapie
                    # Możesz dostosować warunek, aby uwzględniał pewne marginesy
                    unexplored_points.append((i * map.info.resolution, j * map.info.resolution))

        return unexplored_points

    def send_navigation_goal(self, goal_pose):
        self.get_logger().info('Sending navigation goal')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.navigation_client.send_goal_async(goal_msg)

        # Publikuj również na temat /goal_pose
        self.goal_pose_publisher.publish(goal_pose)

def main(args=None):
    rclpy.init(args=args)
    exploration_algorithm = ExplorationAlgorithm()
    rclpy.spin(exploration_algorithm)
    exploration_algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
