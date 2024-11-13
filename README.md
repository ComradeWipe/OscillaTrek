# OscillaTrek
# OscillaTrek: A Smart Walking Stick for the Visually Impaired

OscillaTrek is a smart walking stick project designed to assist visually impaired individuals in navigating their surroundings safely. The walking stick integrates an STM32 microcontroller, ultrasonic sensors, and vibration motors to provide tactile feedback when obstacles are detected.

## Project Overview

The OscillaTrek smart walking stick enhances traditional mobility aids by adding:
- **Obstacle detection** using ultrasonic sensors.
- **Tactile feedback** using an eccentric rotating mass (ERM) motor to notify users of obstacles.
- **STM32-based control system** for sensor data processing and motor control.

### Features

- **Ultrasonic Sensor Integration**: Uses HC-SR04 sensors to detect obstacles up to 2 meters away.
- **Vibration Feedback**: Provides tactile feedback to users when an obstacle is detected within a predefined range.
- **STM32 Control**: Powered by STM32 microcontroller for efficient processing and low power consumption.
  
### Components Used

- **STM32 Microcontroller**: STM32F401RE
- **Ultrasonic Sensors**: HC-SR04
- **Vibration Motor**: ERM (Eccentric Rotating Mass) Motor
- **Breadboard and Dupont Wires**: For prototyping connections
- **Blind Cane Mobility Stick**: Modified to integrate the sensors and control unit

### Getting Started

To get started with this project, you'll need:
- STM32CubeIDE or any compatible IDE.
- STM32CubeMX for generating initial project configuration.
- Basic knowledge of STM32 GPIO configuration and C programming.

### How to Build and Run

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/OscillaTrek.git
   cd OscillaTrek
