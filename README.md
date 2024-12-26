# Zephyr Drone

Zephyr is a lightweight, durable, and versatile drone designed for stability, maneuverability, and various applications. With its minimalist and sleek design, Zephyr combines simplicity with functionality.

## Features

- **Sturdy Build**: Weighing approximately 500g, Zephyr is crafted with PET-G plastic, offering excellent resistance to impact and wear.
- **Efficient Propulsion**: Four arms equipped with brushless motors and two-blade propellers ensure maximum flight time and stability.
- **Enhanced Capabilities**: Designed for diverse use cases, from monitoring to autonomous operations.

![Zephyr Drone](https://github.com/user-attachments/assets/0aa873e7-1434-44ec-a303-b79448b3f561)  
![Zephyr in Action](https://github.com/user-attachments/assets/c92ed451-4680-48d8-9ecc-6158a53d30d5)

### Schematic

![Zephyr Schematic](https://github.com/user-attachments/assets/9630f589-e46a-48ed-8c8e-9ae53c4a65cf)

---

## Components Overview

- **MPU-6050**: 6-axis motion tracking device for acceleration and rotation.
- **BMP-280**: Barometric pressure sensor for altitude and temperature readings.
- **nRF24L01**: Wireless communication module (2.4GHz).
- **Teensy Microcontroller**: ARM Cortex-M based controller for sensor processing and motor control.
- **ESP32-CAM**: Camera module for WiFi-enabled streaming.
- **Electronic Speed Controls (ESCs)**: Motor speed and direction control.
- **Brushless Motors**: Efficient propulsion system.
- **LiPo Batteries**: Power source for the drone.
- **HX-3S-A02**: LiPo battery protection circuit.

---

## Key Enhancements

- **Buzzer**: Added for sound alerts (e.g., low battery, auto-landing, MCU issues).  
- **Power Supply Optimization**: 
  - Removed the LM2596 voltage regulator.
  - Powered components (BMP-280, nRF24L01) via Teensy 4.1’s 3.3V pin.
  - ESC’s 5V output powers most components directly.
- **Voltage Monitoring**: 
  - Removed voltage divider due to inefficiencies with LiPo batteries under high load.
  - Exploring alternatives for battery capacity measurement.

---

## Functionality

- **Sensor Integration**: 
  - MPU-6050 and BMP-280 provide flight data via the I2C protocol.
- **Motor Control**: 
  - Teensy generates PWM signals to adjust motor speed via ESCs.
- **Camera Feed**: 
  - ESP32-CAM transmits video to the ground station over WiFi.
- **Remote Communication**: 
  - nRF24L01 enables real-time data exchange with the Ground Control Station (GCS).
- **Power Management**: 
  - Stable power delivery from LiPo batteries with protection via HX-3S-A02.

---

## Ground Control Station (GCS)

The GCS operates with three roles:

1. **Visual Operator**:  
   - Directly pilots the drone and maintains line of sight.  
   - Ensures safety systems are active, in compliance with REQ-OPS-140.
   - UAV automatically lands if communication is lost (REQ-DES-040).

2. **Remote Operators**:  
   - Monitor UAV telemetry, autonomous operations, and data streams.  
   - Operate from a physical station connected to the UAV via a WiFi network (UDP protocol).

---
