# Gesture-Controlled Wired Mouse

## Overview
An embedded human-computer interaction project that uses an ESP32-C3 and MPU6050 motion sensor to control computer cursor movement through wrist gestures over a wired USB-serial connection.

## Hardware Components
- ESP32-C3
- MPU6050 motion sensor
- Two push buttons
- Connecting wires
- USB cable

## Technologies Used
- Arduino IDE and Embedded C++
- Python
- I²C communication
- USB Serial Communication

## System Workflow
```text
Wrist Gestures
      ↓
MPU6050 Motion Sensing
      ↓
ESP32-C3 Processing
      ↓
USB Serial Communication
      ↓
Python Receiver (Windows)
      ↓
Computer Mouse Control
```

## Working Principle
The MPU6050 measures wrist motion and communicates with the ESP32-C3 through I²C. The microcontroller processes the sensor data and sends mouse commands over USB serial. A Python receiver on Windows interprets these commands to control cursor movement and mouse actions.

## Key Features
- Gesture-based cursor movement
- Left and right click controls
- Double-click to toggle scroll mode
- Two-second button hold to toggle cursor lock
- Motion calibration and direction setup

## Installation and Usage
1. Upload `gesture_mouse.ino` to the ESP32-C3 using Arduino IDE.
2. Enable **USB CDC On Boot** in the board settings.
3. Connect the ESP32-C3 to a Windows computer using USB.
4. On first startup, open Serial Monitor at **115200 baud**, keep the device still during calibration, and follow the direction setup prompts.
5. Close Serial Monitor.
6. Install the Python dependency:

   ```bash
   pip install pyserial
   ```

7. Run the receiver:

   ```bash
   python mouse_receiver.py
   ```

## Applications
- Human-computer interaction
- Embedded systems development
- Motion-controlled interfaces
