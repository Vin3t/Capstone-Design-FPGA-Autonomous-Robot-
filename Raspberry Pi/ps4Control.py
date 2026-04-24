import pygame
import serial
import time

# Serial setup
ser = serial.Serial("/dev/serial0", 38400, timeout=0.1)
addr = 128 # RoboClaw address

# CRC16 function

def crc16(data):
	crc = 0
	for byte in data:
		crc ^= (byte << 8)
		for _ in range(8):
			if crc & 0x8000:
				crc = (crc << 1) ^ 0x1021
			else:
				crc <<= 1
			crc &= 0xFFFF
	return crc

# Send Packet command
def command(cmd, val):
	data = [addr, cmd, val]
	crc = crc16(data)
	packet = data + [crc >> 8, crc & 0xFF]
	ser.write(bytes(packet))

# Map joystick value (-1 to 1) to motor speed (0-127)
def stick_to_speed(value):
	return int(max(0, min(127, abs(int(value * 127)))))

# Motor commands
M1_FORWARD = 0
M1_BACKWARD = 1
M2_FORWARD = 4
M2_BACKWARD = 5

# Pygame setup
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
	print("No controller detected")
	exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"PS4 Controller Conneced: {joystick.get_name()}")

try:
	running = True
	while running:
		pygame.event.pump()

		# Left stick vertical
		left_axis = -joystick.get_axis(4)
		# Right stick vertical
		right_axis = -joystick.get_axis(1)

		left_speed = stick_to_speed(left_axis)
		right_speed = stick_to_speed(right_axis)

		if left_axis >= 0:
			command(M1_FORWARD, left_speed)
		else:
			command(M1_BACKWARD, left_speed)

		if right_axis >= 0:
			command(M2_FORWARD, right_speed)
		else:
			command(M2_BACKWARD, right_speed)

		# PS button to exit
		if joystick.get_button(9):
			running = False

		time.sleep(0.05)

finally:
	# Stop motors on exit
	command(M1_FORWARD, 0)
	command(M2_FORWARD, 0)
	ser.close()
	pygame.quit()
	print("EXIT")
	