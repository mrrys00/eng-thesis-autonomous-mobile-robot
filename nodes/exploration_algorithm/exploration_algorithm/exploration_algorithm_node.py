import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header
from rclpy.action import ActionClient

class ExplorationAlgorithm(Node):

    def __init__(self):
        super().__init__('exploration_algorithm')
        self.get_logger().info('Exploration algorithm is running')

        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Utwórz klienta akcji nawigacyjnej
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        while not self.navigation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Waiting for navigation server...')

    def map_callback(self, msg):
        # Przetwarzaj mapę
        next_point = self.find_next_point(msg)
        if next_point:
            # Jeśli znaleziono nieodwiedzony punkt, planuj trasę do niego
            goal_pose = PoseStamped()
            goal_pose.header = Header()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = next_point[0]
            goal_pose.pose.position.y = next_point[1]
            goal_pose.pose.orientation.w = 1.0

            self.send_navigation_goal(goal_pose)
        else:
            self.get_logger().info('Exploration complete')

    def find_next_point(self, map, margin=100):
        next_point = None

        for i in range(margin, map.info.width - margin):
            for j in range(margin, map.info.height - margin):
                if map.data[i + j * map.info.width] == -1:
                    # Oznacza to nieznaną pozycję na mapie (uwzględniając margines)
                    return (i * map.info.resolution, j * map.info.resolution)

    def send_navigation_goal(self, goal_pose):
        self.get_logger().info('Sending navigation goal')

        # Przygotuj cel akcji nawigacyjnej
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose  # Teraz używamy całej struktury PoseStamped jako celu nawigacyjnego

        # Wyślij żądanie akcji nawigacyjnej
        self.navigation_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    exploration_algorithm = ExplorationAlgorithm()
    rclpy.spin(exploration_algorithm)
    exploration_algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
