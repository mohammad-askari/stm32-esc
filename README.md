# SimpleFOC Implementation on STM32-B-G431B-ESC1 Discovery Kit

## Overview
This repository presents a SimpleFOC-based implementation of Field-Oriented Control (FOC) for the STM32-B-G431B-ESC1 Discovery Kit. Developed for the BLDC actuator module of the Open Dynamic quadruped robot SOLO, this project aims to achieve efficient torque and speed control with an alternative driver due to global chip shortages affecting SOLO's original design. This work addresses the need for high-performance control for applications in bio-inspired quadruped robots.

## Key Features
- **FOC Control**: Utilizes SimpleFOC library for robust position, speed, and torque control.
- **Real-Time Gain Tuning**: Supports tuning through SimpleFOC Studio for optimal performance adjustments.
- **Two-Way UART Communication**: Communicates with the master board, handling feedback at 1 kHz.
- **Optimized Control Rate**: Achieves 7-13 kHz FOC control rates, allowing motor speeds up to 16,000 RPM with enhanced noise reduction and reduced power consumption.

## System Requirements
- **Microcontroller**: STM32-B-G431B-ESC1 Discovery Kit
- **Software**: PlatformIO on Visual Studio Code
- **Motor**: Tested with AT2303 BLDC motor
- **Encoder**: AS5048A magnetic encoder with SPI communication

## Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/mohammad-askari/stm32-esc.git
2.	Open the project in PlatformIO via Visual Studio Code.
3.	Flash the code to your STM32-B-G431B-ESC1 Discovery Kit.

## Usage
- Ensure proper wiring and connection as specified in the documentation.
- Use SimpleFOC Studio for real-time gain tuning and monitoring.
- Connect the STM32 ESC Discovery Kit to the master board via UART at 1 kHz for control data exchange.

## Results
The enhanced FOC implementation demonstrated improved performance with significant gains in speed control, reduced power consumption, and minimal noise, enabling integration with the SOLO robot’s actuator module. The new system’s performance is comparable or superior to the original hardware.

## Acknowledgments
SimpleFOC Community for support and resources.
The project leverages the [SimpleFOC library](https://simplefoc.com) for implementing the FOC control algorithms.
