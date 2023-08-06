

.PHONY
all:
	echo "doing all stuff"

.PHONY
install-gpio:
	sudo apt-get update
	sudo apt-get upgrade -y
	sudo apt-get install rpi.gpio -y
	sudo raspi-config


.PHONY
install-pigpio:
	install-pigpio
	start-pigpio
	test-pigpio

install-pigpio:
	cd ~/Dowlnoads
	wget https://github.com/joan2937/pigpio/archive/master.zip
	unzip master.zip
	cd pigpio-master
	make
	sudo make install
	sudo apt install python-setuptools python3-setuptools -y

start-pigpio:
	sudo pigpiod

kill_pigpio:
	sudo killall pigpiod

test-pigpio:
	sudo ./x_pigpio
	sudo pigpiod
	./x_pigpiod_if2
	./x_pigpio.py
	./x_pigs
	./x_pipe

# need when using C
.PHONY		
install-wiringpi:
	sudo apt-get install wiringpi -y

.PHONY
install-python-deps:
	sudo apt-get install python3-pip
	pip3 install pyserial
