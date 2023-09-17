# ROS Humble on Ubuntu Server on Raspberry PI setup

## 1. Boot the system:

1. Download Ubuntu Server:

        https://ubuntu.com/download/raspberry-pi/thank-you?version=22.04.3&architecture=server-arm64+raspi

2. Write extracted ISO to SD Card
3. Put SD into Raspberry PI
4. Log in:

        login: ubuntu
        password: ubuntu

## 2. Set up network:

1. Check wifi interface name:

        ls /sys/class/net

2. Check the name of the config file:
        
        ls /etc/netplan

3. Open the config file:

        sudo vim /etc/netplan/...-installer-config.yaml
        
4. Add:

        ...
        network:
            +++
            wifis:
                {WIFI_INTERFACE_NAME}:
                optional: true
                    access-points:
                        "{SSID_NAME}":
                            password: "{PASSWORD}"
                    dhcp4: true
            +++
            ...        
		
5. Apply config:

        sudo netplan --debug apply

## 3. (OPTIONAL) Connect from another computer

1. Check IP address of the Raspberry PI:

        hostname -I

2. SSH from another computer on the same network:

        ssh ubuntu@{IP}

## 3. Install ROS Humble:

1. Add Universe repository:

        sudo apt install software-properties-common
        sudo add-apt-repository universe

2. Add the ROS 2 GPG key:

        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

3. Add the repository to sources list:
 
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

4. Update system:

        sudo apt update && sudo apt upgrade && sudo apt autoremove

5. Install ROS Humble and development tools:
        
        sudo apt install ros-humble-ros-base ros-dev-tools

6. Source the setup script:
        
        source /opt/ros/humble/setup.bash
        echo "source /opt/ros/humble/setup.bash" >> .bashrc

## 4. Install Nodes

1. SLAM Toolbox:

        sudo apt install ros-humble-slam-toolbox

2. URG Node:

        mkdir ros2_workspace ros2_workspace/src
        cd ros2_workspace/src
        git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git
        rosdep update
        rosdep install -i --from-paths urg_node2
        cd ..
        colcon build --symlink-install
        source install/setup.bash

## 7. Edit Config

1. Open URG Node config file:

        vim ros2_workspace/src/urg_node2/launch/urg_node2.launch.py

2. Change:

        ...
        config_file_path = os.path.join(
            get_package_share_directory('urg_node2'),
            'config',
            ---
            'params_ether.yaml'
            ---
            +++
            'params_serial.yaml'
            +++
            )
        ...

## 6. Run Nodes

1. SLAM Toolbox:

        ros2 run slam_toolbox async_slam_toolbox_node

2. URG Node - open new SSH connection, or new TTY if connected directly (Ctrl + Alt + F2)

        cd ros2_workspace
        source install/setup.bash
        ros2 launch urg_node2 urg_node2.launch.py

3. TODO...