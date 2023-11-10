# Prepare Ubuntu 22.04 LTS desktop VM

Why it may be important?\
It's much easier to develop new ROS feaures with GUI and better PC, for example RPi resources are to small to use rViz visualization.

## Install VirtualBox

[VirtualBox page](https://www.virtualbox.org)

## Prepare your VM

1. Download Ubuntu 22.04 LTS, [page](https://releases.ubuntu.com/jammy/), \*.iso
2. Suggested machine settings:
   - 8192 MB RAM
   - about 50 GB Hard Disk
   - 4 processors
   - simple login and password
3. After creating machine go to `Settings >> Network >> Adapter 1`
   - Attached to: NAT
   - Port Forwarding:
     - Name: SSH
     - Protocol: TCP
     - Host IP: 127.0.0.1
     - Host Port: 2222
     - Guest IP: `<your VMs IP>` (check using `hostname -I` in Ubuntu's terminal)
     - Guest Port: 22
   - This is because we **want** use SSH on out primary (host) machine
4. Reboot VMs system
5. Setup Ubuntu

   - [add sudo privileges](https://www.baeldung.com/linux/username-not-in-sudoers-file)
   - install `sudo apt install openssh-server` and enable `sudo systemctl enable ssh` openssh-server, easy [manual](https://www.cyberciti.biz/faq/how-to-install-ssh-on-ubuntu-linux-using-apt-get/)
   - in the `/etc/ssh/sshd_config` uncomment lines

     ```
         Port 22
         AddressFamily any
         ListenAddress 0.0.0.0
         ListenAddress ::
     ```

   - reload sshd: `sudo systemctl force-reload sshd`
   - verify if machine is set on 22 port `sudo netstat -tlnp | grep ssh`

6. Install openssh-client (according to your host system)
7. Connect to ssh on your host machine `ssh -p 2222 <your_username>@127.0.0.1`
8. Install packeges
    - pip3 `sudo apt install python3-pip`
    - vs code (if not installed yet)

__NOTE__: To copy files (eg. ROS nodes) via scp use command `scp -P 2222 -r <your_username>@127.0.0.1:<your_source_path> <your_dest_path>`

## Setup ROS2 Humble on the VM

### Preinstall

```sh
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt upgrade -y && sudo apt autoremove
    sudo apt install ros-humble-ros-base ros-dev-tools -y
    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> .bashrc
```

### Prepare workspace

```sh
    mkdir -p ~/ros2_workspace/src/
```

### Install ROS packages necessary to the simulation

```sh
    sudo apt update
    sudo apt install ros-humble-turtlebot3-bringup ros-humble-turtlebot3-description ros-humble-turtlebot3-teleop ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-cyclonedds-cpp ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp ros-humble-turtlebot3* 
```

### Create custom ROS package for data processing

```sh
    cd ros2_workspace/src/
    ros2 pkg create --build-type ament_python python_data_processor
```

Install proper python dependencies:

```sh
pip3 install setuptools==58.2.0
```

Edit the **package.xml** in **python_data_processor/** and add:

```xml
<depend>std_msgs</depend>
```

Create **data_processor_node.py** code into **python_data_processor/python_data_processor**.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Change this to the appropriate message type

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor_node')

        # Publisher on topic2
        self.publisher_ = self.create_publisher(String, 'topic2', 10)

        # Subscriber on topic1
        self.subscription = self.create_subscription(
            String,
            'topic1',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Processing the data
        processed_data = self.process_data(msg.data)

        # Publishing the processed data
        self.publisher_.publish(String(data=processed_data))
        self.get_logger().info(f"Received {msg.data}, Published {processed_data}")

    def process_data(self, data):
        # An example data processing function.
        # Modify this according to your needs.
        return data.upper()  # Converts the string to uppercase

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Modify the **setup.py** in **python_data_processor/** to look like this:

```python
from setuptools import setup

setup(
    name='python_data_processor',
    version='0.0.0',
    packages=[],
    py_modules=[
        'python_data_processor.data_processor_node'
    ],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/python_data_processor']),
        ('share/python_data_processor', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'data_processor = python_data_processor.data_processor_node:main'
        ],
    },
)
```

Building and sourcing:

```
cd ~/ros2_workspace/
colcon build
source install/setup.bash
```

<!-- ### Create custom ROS package for goal finder (not tested yet)

```sh
    cd ros2_workspace/src/
    ros2 pkg create --build-type ament_python extrapolation_algotihm
```

<!-- Install proper python dependencies:

```sh
pip3 install setuptools==58.2.0
``` -->

<!-- Edit the **package.xml** in **python_data_processor/** and add:

```xml
    <exec_depend>ament_index_python</exec_depend>
    <exec_depend>rclpy</exec_depend>
    <exec_depend>nav2_msgs</exec_depend>
    <exec_depend>geometry_msgs</exec_depend>
    <exec_depend>nav_msgs</exec_depend>
    <exec_depend>std_msgs</exec_depend>

    <export>
    <build_type>ament_python</build_type>
    </export>
```

Create **data_processor_node.py** code into **python_data_processor/python_data_processor**.

```python
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
        while not self.navigation_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for navigation server...')

    def map_callback(self, msg):
        # Przetwarzaj mapę
        unexplored_points = self.find_unexplored_points(msg)
        if unexplored_points:
            # Jeśli znaleziono nieodwiedzone punkty, planuj trasę do pierwszego z nich
            goal_pose = PoseStamped()
            goal_pose.header = Header()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = unexplored_points[0][0]
            goal_pose.pose.position.y = unexplored_points[0][1]
            goal_pose.pose.orientation.w = 1.0

            self.send_navigation_goal(goal_pose)
        else:
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

def main(args=None):
    rclpy.init(args=args)
    exploration_algorithm = ExplorationAlgorithm()
    rclpy.spin(exploration_algorithm)
    exploration_algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
``` -->

<!-- Modify the **setup.py** in **python_data_processor/** to look like this:

```python
from setuptools import setup

setup(
    name='python_data_processor',
    version='0.0.0',
    packages=[],
    py_modules=[
        'python_data_processor.data_processor_node'
    ],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/python_data_processor']),
        ('share/python_data_processor', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'data_processor = python_data_processor.data_processor_node:main'
        ],
    },
)
``` -->
<!-- 
Building and sourcing:

```
cd ~/ros2_workspace/
colcon build
source install/setup.bash
``` -->

Copy via scp the necessary node - described in nodes/README.md

### Launch them all!

In the separate terminals:

```sh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch nav2_bringup rviz_launch.py
ros2 run slam_toolbox async_slam_toolbox_node   # probably need additional configuration to make sure that it will create scnas only with actual odometry
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: 'path_to_non_yet_existing_new_map'}"
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false autostart:=true map:=path_to_created_before_new_map.yaml
ros2 run python_data_processor data_processor   # TO DO process data from nav2_wayland_follower and send directly to miabot
ros2 run extrapolation_algotihm exploration_algorithm_node  # TO DO 
```

### Useful ROS commands

```sh
ros2 node list -a
ros2 topic list
ros2 topic pub /topic1 std_msgs/String "data: aatest msg2"
ros2 topic echo /topic1
…
```
