# Prepare Ubuntu 22.04 LTS desktop VM 

Why it may be important?\
It's much easier to develop new ROS feaures with GUI and better PC, for example RPi resources are to small to use rViz visualization.

## Install VirtualBox

[VirtualBox page](https://www.virtualbox.org)

## Prepare your VM

1. Download Ubuntu 22.04 LTS, [page](https://releases.ubuntu.com/jammy/), *.iso
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

## Setup ROS2 Humble on the VM

### Preinstall

    ```
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	    sudo apt update && sudo apt upgrade -y && sudo apt autoremove
	    sudo apt install ros-humble-ros-base ros-dev-tools -y
	    source /opt/ros/humble/setup.bash
	    echo "source /opt/ros/humble/setup.bash" >> .bashrc
    ```

### to be continued …