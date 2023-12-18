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
9. Add change owner to *~/.bashrc*:
    ```
        echo <password> | sudo -S chown $USER /dev/ttyACM0 /dev/ttyS0
    ```

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

Copy via scp the necessary node - described in nodes/README.md

### Launch them all!

In the separate terminals:

```sh
# Don't run
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
ros2 run python_data_processor data_processor   # TO DO process data from nav2_wayland_follower and send directly to miabot

# run on VM / PC
ros2 launch nav2_bringup rviz_launch.py
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file slam.yaml
ros2 launch nav2_bringup navigation_launch.py params_file:=nav2_params2.yaml
ros2 run exploration_algotihm exploration_algorithm_node  # TO DO 

# run on RPi
ros2 launch urg_node2 urg_node2.launch.py
ros2 run miabot_node miabot_node # TO DO
```

### Useful ROS commands

```sh
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: 'path_to_non_yet_existing_new_map'}"
ros2 node list
ros2 topic list
ros2 topic pub /topic1 std_msgs/String "data: aatest msg2"
ros2 topic echo /topic1
ros2 run tf2_tools view_frames
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.01}, angular: {z: 0.0}}"
ros2 param get /slam_toolbox base_frame
ros2 param set /slam_toolbox base_frame base_link
colcon build --packages-select miabot_node
…
```
