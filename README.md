# 🤖 Eurobot 2025 SIMA

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## 📋 Overview

SIMA (Small Independent Mobile Actuators) is an autonomous robot developed for the Eurobot 2025 competition. The robot uses advanced sensing, navigation, and obstacle avoidance capabilities to complete various competition tasks autonomously.

## ✨ Features

- **Autonomous Navigation**: Position tracking and pathfinding to specific coordinates
- **Obstacle Detection & Avoidance**: Using VL53L0X ToF (Time of Flight) sensors
- **Wireless Control**: ESP-NOW communication for robot control and status updates
- **Web Interface**: Real-time monitoring and control via browser
- **OTA Updates**: Wireless firmware updates through ElegantOTA
- **LED Status Indication**: Visual feedback of robot state through RGB LEDs

## 🛠️ Hardware Components

- ESP32-S3 Microcontroller
- VL53L0X Time-of-Flight Distance Sensors (x3)
- Stepper Motors with Drivers
- Servo Motors
- RGB LED Strip (WS2812)

## 📌 Pin Configuration & Functionality

### 📏 VL53L0X Sensors
- SDA: GPIO 8 - I2C Data line for communication with all VL53L0X sensors
- SCL: GPIO 9 - I2C Clock line for communication with all VL53L0X sensors
- XSHUT1: GPIO 10 - Shutdown pin for first VL53L0X sensor (Middle sensor)
- XSHUT2: GPIO 11 - Shutdown pin for second VL53L0X sensor (Left sensor)
- XSHUT3: GPIO 12 - Shutdown pin for third VL53L0X sensor (Right sensor)

These sensors provide obstacle detection in three directions, enabling the robot to navigate around obstacles and make intelligent path decisions.

### ⚙️ Stepper Motors
- MS1: GPIO 4 - Microstep resolution configuration pin 1
- MS2: GPIO 5 - Microstep resolution configuration pin 2
- STEP_PIN_L: GPIO 6 - Step pulse pin for left motor (each pulse causes one step)
- DIR_PIN_L: GPIO 7 - Direction control pin for left motor (HIGH/LOW determines direction)
- STEP_PIN_R: GPIO 15 - Step pulse pin for right motor
- DIR_PIN_R: GPIO 16 - Direction control pin for right motor

The stepper motors provide precise motion control with variable speed through acceleration/deceleration algorithms.

### 🦾 Servo Motors
- Servo Right: GPIO 19 - PWM control for right gripper servo
- Servo Left: GPIO 20 - PWM control for left gripper servo

These servos are used for manipulating game elements during competition.

### 💡 RGB LED Strip
- Data Pin: GPIO 3 - Data line for controlling WS2812 addressable LED strip

The LED strip provides visual feedback about the robot's operational state through various animation patterns:
- 🌈 **Rainbow Effect**: Default mode - colorful cycling pattern during normal operation
- 🔵 **Blue Breathing**: Firmware update in progress
- 🔴 **Red Breathing**: Low battery warning
- 🟢 **Green Breathing**: WiFi setup mode (NetWizard active)
- 🟠 **Orange Breathing**: WiFi disconnected
- ✨ **Custom Breathing**: Pulsing animation with configurable colors
- 🎨 **Color Wipe**: Sequential filling animation with configurable colors
- 🔆 **Static Color**: Solid color display for status indication

The non-blocking implementation ensures LED animations don't interfere with critical control tasks.

## 💻 Software Architecture

The software is organized into the following key modules:

- 🧠 **`sima_core`**: Main robot control logic, navigation algorithms
- 👁️ **`VL53L0X_Sensors`**: Distance sensors management
- 🚶 **`motion_control`**: Movement functions (forward, backward, turns)
- 📡 **`esp_now_comm`**: Wireless communication
- 🌐 **`web_interface`**: Web-based control and monitoring dashboard
- 💫 **`led_control`**: Status LED management

The system implements a multi-task architecture using FreeRTOS to manage different subsystems in parallel.

## 🚀 Getting Started

### 📋 Prerequisites

- PlatformIO IDE
- ESP32-S3 compatible board
- Required hardware components (sensors, motors, etc.)

### 🔧 Installation

1. Clone this repository:
   ```
   git clone https://github.com/DIT-ROBOTICS/Eurobot-2025-SIMA.git
   ```

2. Open the project in PlatformIO IDE

3. Configure the robot parameters in `include/config.h` if needed

4. Build and upload the firmware:
   ```
   platformio run --target upload
   ```

### ⚙️ Initial Setup

1. Power on the robot
2. Connect to the WiFi network created by the robot (name: DIT-SIMA-XX)
3. Navigate to http://dit-sima-xx.local/ or the IP address shown in serial monitor
4. Configure the robot's WiFi connection through the NetWizard interface

## 📱 Usage

### 🌐 Web Interface Control

The web interface provides:
- Real-time sensor readings
- Position data (x, y, θ)
- Manual control buttons
- Status monitoring
- OTA update capability

#### 🔗 Web Interface Entry Points

- **Main Dashboard**: http://dit-sima-xx.local or http://[IP_ADDRESS]/
  - Control panel with sensor readings and position information
  - Manual control buttons for robot movement
  - Status indicators

- **OTA Updates**: http://dit-sima-xx.local/update or http://[IP_ADDRESS]/update
  - Firmware update interface
  - Upload new binary files for wireless updates
  - Update progress monitoring

- **WebSerial Console**: http://dit-sima-xx.local/webserial or http://[IP_ADDRESS]/webserial
  - Real-time debugging console
  - Command input interface
  - Log viewing for diagnostics

- **WiFi Configuration**: http://192.168.4.1
  - WiFi network configuration
  - Connection status and signal strength
  - Network scanning interface

### 📶 ESP-NOW Control

The robot can receive commands wirelessly through ESP-NOW protocol.

## 👨‍💻 Development

### 📁 Code Structure

```
├── include/
│   └── config.h          # Configuration and pin definitions
├── lib/
│   ├── esp_now_comm/     # Wireless communication
│   ├── led_control/      # RGB LED management
│   ├── motion_control/   # Movement functions
│   ├── sima_core/        # Main robot control logic
│   ├── VL53L0X/          # VL53L0X sensor driver
│   ├── VL53L0X_Sensors/  # High-level sensors management
│   └── web_interface/    # Web dashboard
└── src/
    └── main.cpp          # Application entry point
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.