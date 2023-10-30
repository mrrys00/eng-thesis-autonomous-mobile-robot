.PHONY: prepare_ubuntu
prepare_ubuntu:
	hostname -I

	sudo apt install software-properties-common net-tools curl
	sudo add-apt-repository universe

.PHONY: install_ros
install_ros:
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	sudo apt update && sudo apt upgrade -y && sudo apt autoremove
	sudo apt install ros-humble-ros-base ros-dev-tools -y
	source /opt/ros/humble/setup.bash
	echo "source /opt/ros/humble/setup.bash" >> .bashrc

