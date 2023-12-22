# This makefile allows to prepare ros enviromnent for Miabot RPi and VM (or PC)
# Make sure all your systems are ubuntu 22.04 LTS 

MAKEFLAGS += --no-print-directory

# to verify UBINTU_CODENAME out of makefile use `. /etc/os-release && echo $UBUNTU_CODENAME`
UBUNTU_CODE = jammy
ARCH = `dpkg --print-architecture`
SUDO_PASSWORD = 12345678

ROS2_WORKSPACE = ./ros2_workspace/
PROJECT_ROOT = ./

NODES = nodes/
NODE_MIABOT = miabot_node/
NODE_EXPLORATION = exploration_algorithm/
NODE_PROJECT_BRINGUP = project_bringup/

SRC = src/
STARTUP = startup/


.SILENT: perpare_robot prepare_pc

test:
	echo $(USER)
	echo $(ROS2_WORKSPACE)
	echo $(PROJECT_ROOT)
	echo $(UBUNTU_CODE)
	echo $(ARCH)

add_serial_port_privileges:
	echo $(SUDO_PASSWORD) | sudo -S chown $(USER) /dev/ttyACM0 /dev/ttyS0

install_ros2_humble:
	sudo apt install software-properties-common -y
	sudo add-apt-repository universe -y
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(ARCH) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(UBUNTU_CODE) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	sudo apt update && sudo apt upgrade -y && sudo apt autoremove
	sudo apt install ros-humble-ros-base ros-dev-tools -y
	# source /opt/ros/humble/setup.bash
	# echo "source /opt/ros/humble/setup.bash" >> .bashrc

install_ros2_nodes:
	sudo apt update
	sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp
	
prepare_python_dependencies:
	sudo apt install python3-pip
	pip3 install setuptools==58.2.0

prepare_ros2_workspace:
	mkdir -p $(ROS2_WORKSPACE)$(SRC)
	cd $(ROS2_WORKSPACE)

prepare_urg2_node:
	cd $(ROS2_WORKSPACE)src/; \
	git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git; \
	rosdep update; \
	rosdep install -i --from-paths urg_node2

copy_nodes:
	cp -r $(PROJECT_ROOT)$(NODES)$(NODE_PROJECT_BRINGUP) $(PROJECT_ROOT)$(NODES)$(NODE_MIABOT) $(PROJECT_ROOT)$(NODES)$(NODE_EXPLORATION) $(ROS2_WORKSPACE)src/

build_ros2_workspace:
	cd $(ROS2_WORKSPACE); \
	colcon build


# these steps manually :)

# run_robot_nodes:
# 	cd $(ROS2_WORKSPACE)
# 	source install/setup.bash
# 	ros2 launch project_bringup robot_bringup.launch.py

# run_pc_nodes:
# 	cd $(ROS2_WORKSPACE)
# 	source install/setup.bash
# 	ros2 launch project_bringup pc_bringup.launch.py


prepare_robot:
	$(MAKE) install_ros2_humble
	$(MAKE) add_serial_port_privileges
	$(MAKE) prepare_ros2_workspace
	$(MAKE) copy_nodes
	$(MAKE) prepare_urg2_node
	$(MAKE) build_ros2_workspace

prepare_pc:
	$(MAKE) install_ros2_humble
	$(MAKE) install_ros2_nodes
	$(MAKE) prepare_ros2_workspace
	$(MAKE) copy_nodes
	$(MAKE) build_ros2_workspace
