# Remote Controller with ESP-NOW

This project implements a remote controller using ESP-NOW communication with an ESP32. The remote sends directional commands and handles sensor feedback such as air quality, temperature, and obstacle detection.

## Features
- **Directional Control**: Forward, backward, left, and right buttons
- **LED Indicators**:
    - Red and green LEDs for gas quality warning
    - Yellow LED for obstacle detection
- **ESP-NOW Communication**: Send commands to a robot and receive sensor feedback
- **Obstacle Detection**: Warns when attempting forward movement into an obstacle

## Pin Connections

| Pin Number | Function | Description |
|------------|----------|-------------|
| 16 | BTN_FORWARD | Button for forward movement |
| 17 | BTN_BACKWARD | Button for backward movement |
| 18 | BTN_LEFT | Button for left movement |
| 19 | BTN_RIGHT | Button for right movement |
| 22 | GAS_LED_P1 | Red LED for gas warning |
| 23 | GAS_LED_P2 | Green LED for normal air quality |
| 21 | WARNING_LED | Yellow LED for obstacle detection warning |

## Setup

### Hardware
1. Connect buttons to the respective pins with pull-up resistors
2. Connect LEDs to the specified pins with appropriate resistors

### Software
1. Ensure you have the ESP32 board package installed in the Arduino IDE
2. Replace `botAddress` with the MAC address of the target robot

### Dependencies
- ESP-NOW library (included in ESP32 package)

## Usage

1. **Upload the Code**
     - Compile and upload the sketch to your ESP32 board
2. **Pair Devices**
     - Ensure the robot's ESP32 is configured to receive commands
3. **Control**
     - Press the buttons to control movement
     - LEDs will indicate gas quality and obstacle presence

## Implementation Details

### Button Handling
- Checks the state of directional buttons
- Sends the appropriate command using the CommandData structure

### LED Indicators
- **Gas Quality**:
    - Red LED: Air quality exceeds threshold
    - Green LED: Normal air quality
- **Obstacle Warning**:
    - Blinking: Forward movement attempted with obstacle
    - Solid: Obstacle detected, no movement

### Sensor Data
Processes feedback including:
- Temperature
- Humidity
- Air quality
- Obstacle detection
- Light levels

### Constants
- `BLINK_INTERVAL`: 500ms (LED warning blink)
- `GAS_THRESHOLD`: Air quality warning threshold

## Notes
- Keep devices within ESP-NOW range
- Calibrate gas sensor (MQ135) for accuracy