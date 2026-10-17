# Gesture-Controlled Wired Mouse

## Overview
A gesture-based mouse system that uses an ESP32-C3 and MPU6050 motion sensor to translate hand movements into computer cursor control.

## Hardware Components
- ESP32-C3
- MPU6050 motion sensor
- Push buttons
- Connecting wires
- USB cable

## Technologies Used
- Arduino IDE
- Embedded C++
- I²C communication
- Motion sensing

## System Workflow

```text
Hand Movement
      ↓
MPU6050 Motion Sensing
      ↓
ESP32-C3 Data Processing
      ↓
Mouse Control Commands
      ↓
Wired Communication
      ↓
Computer Cursor Control
```

## Working Principle
The MPU6050 measures hand movement and sends motion data to the ESP32-C3 through I²C. The microcontroller processes the readings and generates control commands for cursor movement and button operations. The computer receives these commands through the implemented wired interface.

## Key Features
- Gesture-based cursor movement
- Physical button controls
- Wired computer connectivity
- Motion-based interaction

## Applications
- Human-computer interaction
- Embedded systems projects
- Motion-controlled interfaces
