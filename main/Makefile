PORT = /dev/ttyACM0

compile:
	arduino-cli compile --fqbn arduino:avr:mega

upload:
	arduino-cli upload -p $(PORT)

serial:
	rosrun rosserial_arduino serial_node.py _port:=$(PORT)