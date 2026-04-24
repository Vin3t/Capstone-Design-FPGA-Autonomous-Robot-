# Raspberry Pi Subsystem

This directory contains the high-level software for the Raspberry Pi, which serves as the central user interface and communication hub for the autonomous service robot. The system handles manual motor control via a PlayStation 4 controller and provides a touchscreen graphical user interface for setting navigation goals and transmitting map data to the Nexys A7 FPGA.

## File Overview

* **`ps4_motor_control.py`**: A manual drive script that uses the Pygame library to read inputs from a connected PS4 controller. It maps the joystick axes to motor speeds and sends CRC16-validated packet commands to a RoboClaw motor controller over UART.
* **`robot_gui.py`**: A Tkinter touchscreen application. It displays a 20x20 navigation grid, manages simulated LiDAR obstacle mapping, captures user goal inputs, and handles bidirectional UART communication with the FPGA for hardware-accelerated A* pathfinding.


## Hardware Requirements

* **Raspberry Pi**  + **Touchscreen display recommended**
* **PS4 DualShock Controller** (Connected via USB)
* **RoboClaw Motor Controller** (Address configured to `128`)
* **Nexys A7 FPGA** (Connected via UART for pathfinding)

## System Setup and Dependencies

Before executing the scripts, ensure the Raspberry Pi's hardware UART is enabled and the serial console is disabled. 

1. Open the Raspberry Pi configuration tool in the terminal:
   ```bash
   sudo raspi-config

2. Navigate to Interface Options -> Serial Port.

3. Select No when asked if a login shell should be accessible over serial.

4. Select Yes when asked if the hardware serial port should be enabled.

5. Reboot the Raspberry Pi.

## Python Dependencies
Install the required libraries using pip:
- pip install pygame pyserial pillow numpy

## Instructions
1. Manual PS4 Control (ps4_motor_control.py)
Ensure the PS4 controller is paired to the Raspberry Pi. The script maps the left and right vertical joystick axes to independent tank-drive motor speeds.

- python3 ps4_motor_control.py

Left Stick (Vertical): Controls Motor 1 (Forward/Backward)

Right Stick (Vertical): Controls Motor 2 (Forward/Backward)

PS Button: Exits the script and safely halts the motors.

2. Autonomous Navigation GUI (robot_gui.py)
Run the GUI within the Raspberry Pi desktop environment.

- python3 robot_gui.py

## GUI Workflow:

Load LiDAR: Generates a map of obstacles (currently simulated for testing purposes).

Set Goal: Tap an empty cell on the 20x20 grid to mark the destination.

SEND & RUN: Transmits the 404-byte payload to the FPGA.

Unlock: Once the FPGA reports a successful arrival (0xBB), the unlock button becomes active, allowing the user to release the cargo.

## FPGA UART Protocol Specification
The robot_gui.py script communicates with the Nexys A7 FPGA using a strict 404-byte UART packet operating at 115200 baud. The FPGA automatically triggers the A* search on the rising edge of the final received map byte.

Transmitted Packet (Raspberry Pi -> FPGA)

Byte Index          Description             Value Range
0                   Start Row               0 - 19
1                   Start Column            0 - 19
2                   Goal Row                0 - 19
3                   Goal Column             0 - 19
4 to 403            Obstacle Map (Row)      0x00 (Free), 0x01 (Obstacle)

Status Replies (FPGA -> Raspberry Pi)
The GUI runs a background thread that polls for 1-byte status updates from the FPGA:

Byte Received           Meaning             System Action
0xAA                    Path Found          Updates the GUI status; motors engage.
0xFF                    No Path Exists      Prompts the user to replan the goal.
0xBB                    Arrived at Target   Navigation completes; enables the delivery lock release.