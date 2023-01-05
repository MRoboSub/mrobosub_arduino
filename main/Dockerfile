FROM mrobosub/mrobosub:latest
RUN apt update &&\
    apt upgrade -y &&\
    apt install -y ros-noetic-rosserial-arduino &&\
    apt install -y ros-noetic-rosserial &&\
    apt install -y curl
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/bin sh &&\
    arduino-cli config init &&\
    arduino-cli core update-index &&\
    arduino-cli core install arduino:avr &&\
    arduino-cli lib install "BlueRobotics MS5837 Library" &&\
    arduino-cli lib install "ArduinoThread" &&\
    arduino-cli lib install "Servo" &&\
    arduino-cli config set library.enable_unsafe_install true &&\
    arduino-cli lib install --git-url https://github.com/mikaelpatel/Arduino-Scheduler
RUN ["/bin/bash", "-ic", "rosrun rosserial_arduino make_libraries.py /root/Arduino/libraries/"]
