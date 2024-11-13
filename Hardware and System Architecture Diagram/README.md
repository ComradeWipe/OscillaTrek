# OscillaTrek System Architecture

## Overview
The OscillaTrek project is designed to showcase an interconnected system using STM32 microcontrollers, sensors, and a vibration motor. This document will walk you through the hardware architecture and explain how different components are connected and interact with each other.

## System Components
- **STM32 Microcontrollers**: The project utilizes two STM32 microcontrollers to manage the sensing, communication, and control aspects of the system.
- **Ultrasonic Sensors**: These are used to detect distance or obstacles and are connected to one of the STM32 microcontrollers.
- **Vibration Motor**: A UX Cell vibration motor is used as the output component, controlled by the second STM32 microcontroller.
- **Laptop**: Both STM32 microcontrollers are connected to a laptop for development, data acquisition, and monitoring.

## Figures and Descriptions

### Figure 1: Sensor Connection with STM32
This figure illustrates **STM32#1** connected to two ultrasonic sensors (**Sensor1** and **Sensor2**). These sensors are used for distance measurement. The STM32 microcontroller collects data from both sensors and processes it for further communication. The sensors are connected to specific GPIO pins on STM32#1, which handle triggering and receiving echo signals.

### Figure 2: Vibration Motor Setup
**STM32#2** is connected to a **UX Cell Vibration Motor**, which serves as the output mechanism. Based on the distance data received from STM32#1, STM32#2 activates the vibration motor when certain conditions are met. This setup provides feedback based on the sensor inputs, simulating proximity-based alerting functionality.

### Figure 3: Communication Between STM32 Microcontrollers
This figure shows the communication link between **STM32#1** and **STM32#2**. The microcontrollers are interconnected through UART communication, with **VSART2_TX** and **VSART2_RX** pins used for sending and receiving data. This connection allows STM32#1 to send processed data (such as sensor measurements) to STM32#2, which then takes action, such as triggering the vibration motor. Ground (GND) is also connected to ensure proper signal reference.

### Figure 4: Complete System Overview
This figure presents the complete system architecture. It shows both STM32 microcontrollers, their connections to the sensors, vibration motor, and their communication link. The diagram also includes the laptop used to program, monitor, and analyze the system. STM32#1 collects distance data using the sensors, and this data is communicated to STM32#2, which controls the vibration motor accordingly.

## System Workflow
1. **Data Collection**: **STM32#1** collects distance measurements from **Sensor1** and **Sensor2**.
2. **Communication**: The distance data is sent from **STM32#1** to **STM32#2** via UART communication.
3. **Decision Making**: **STM32#2** processes the data and, if a specific condition is met (e.g., distance below a threshold), activates the **vibration motor**.
4. **Feedback**: The vibration motor provides haptic feedback based on the data received.

## Purpose and Use Cases
The architecture presented here can be used in various applications such as proximity alert systems, haptic feedback mechanisms, and general IoT setups where sensor data needs to be processed and acted upon by multiple interconnected devices.

## Future Development
- Implementing advanced filtering algorithms to reduce noise in sensor data.
- Developing a more robust communication protocol between STM32 microcontrollers to handle potential packet loss.
- Integrating additional feedback mechanisms, such as LED indicators, for more intuitive output.

## Conclusion
The OscillaTrek system demonstrates a practical implementation of sensor-based interaction using multiple microcontrollers and feedback mechanisms. The diagrams provided offer a clear visualization of the system's hardware setup, making it easier to understand the relationships between components and their roles within the project.


